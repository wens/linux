/*
 * RSB (Reduced Serial Bus) driver.
 *
 * Author: Chen-Yu Tsai <wens@csie.org>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * The RSB controller looks like an SMBus controller which only supports byte
 * and word data transfers. But, it differs from standard SMBus protocol on 
 * several aspects:
 * - the line protocol is very different with a push-pull interface
 * - devices are sent a special command to switch them to RSB mode
 * - it supports assigning at most 15 runtime addresses to slave devices
 * - read/write addressing of slave device uses runtime addresses
 * - it adds a parity bit every 8bits of data and address
 * - only one read access is required to read a byte (instead of a write
 *   followed by a read access in standard SMBus protocol)
 *
 * This means this bus cannot be used to interface with standard SMBus
 * devices. Devices known to support this interface include the AXP223,
 * AXP809, and AXP806 PMICs, and the AC100 audio codec.
 *
 * This driver is based on i2c-sun6i-p2wi.c, the P2WI bus driver.
 *
 */
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>


/* RSB registers */
#define RSB_CTRL	0x0	/* Global control */
#define RSB_CCR		0x4	/* Clock control */
#define RSB_INTE	0x8	/* Interrupt controls */
#define RSB_INTS	0xc	/* Interrupt status */
#define RSB_ADDR	0x10	/* Address to send with read/write command */
#define RSB_DATA	0x1c	/* Data to read/write */
#define RSB_LCR		0x24	/* Line control */
#define RSB_DMCR	0x28	/* Device mode (init) control */
#define RSB_CMD		0x2c	/* RSB Command */
#define RSB_DAR		0x30	/* Device address / runtime address */

/* CTRL fields */
#define RSB_CTRL_START_TRANS		BIT(7)
#define RSB_CTRL_ABORT_TRANS		BIT(6)
#define RSB_CTRL_GLOBAL_INT_ENB		BIT(1)
#define RSB_CTRL_SOFT_RST		BIT(0)

/* CLK CTRL fields */
#define RSB_CCR_SDA_OUT_DELAY(v)	(((v) & 0x7) << 8)
#define RSB_CCR_MAX_CLK_DIV		0xff
#define RSB_CCR_CLK_DIV(v)		((v) & RSB_CCR_MAX_CLK_DIV)

/* STATUS fields */
#define RSB_INTS_TRANS_ERR_ACK		BIT(16)
#define RSB_INTS_TRANS_ERR_ID(v)	(((v) >> 8) & 0xf)
#define RSB_INTS_LOAD_BSY		BIT(2)
#define RSB_INTS_TRANS_ERR		BIT(1)
#define RSB_INTS_TRANS_OVER		BIT(0)

/* LINE CTRL fields*/
#define RSB_LCR_SCL_STATE		BIT(5)
#define RSB_LCR_SDA_STATE		BIT(4)
#define RSB_LCR_SCL_CTL			BIT(3)
#define RSB_LCR_SCL_CTL_EN		BIT(2)
#define RSB_LCR_SDA_CTL			BIT(1)
#define RSB_LCR_SDA_CTL_EN		BIT(0)

/* DEVICE MODE CTRL field values */
#define RSB_DMCR_DEVICE_START		BIT(31)
#define RSB_DMCR_MODE_DATA		(0x7c << 16)
#define RSB_DMCR_MODE_REG		(0x3e << 8)
#define RSB_DMCR_DEV_ADDR		0x00

/* CMD values */
#define RSB_CMD_RD8			0x8b
#define RSB_CMD_RD16			0x9c
#define RSB_CMD_RD32			0xa6
#define RSB_CMD_WR8			0x4e
#define RSB_CMD_WR16			0x59
#define RSB_CMD_WR32			0x63
#define RSB_CMD_STRA			0xe8

/* DAR fields */
#define RSB_DAR_RTA(v)			(((v) & 0xff) << 16)
#define RSB_DAR_DA(v)			((v) & 0xffff)

#define RSB_MAX_FREQ			20000000

#define RSB_CTRL_NAME			"sunxi-rsb"

struct rsb {
	struct i2c_adapter adapter;
	struct completion complete;
	unsigned int status;
	void __iomem *regs;
	struct clk *clk;
	struct reset_control *rstc;
};

static irqreturn_t rsb_interrupt(int irq, void *dev_id)
{
	struct rsb *rsb = dev_id;
	unsigned long status;

	status = readl(rsb->regs + RSB_INTS);
	rsb->status = status;

	/* Clear interrupts */
	status &= (RSB_INTS_LOAD_BSY | RSB_INTS_TRANS_ERR |
		   RSB_INTS_TRANS_OVER);
	writel(status, rsb->regs + RSB_INTS);

	complete(&rsb->complete);

	return IRQ_HANDLED;
}

/* common code that starts a transfer */
static int rsb_run_xfer(struct rsb *rsb)
{
	if (readl(rsb->regs + RSB_CTRL) & RSB_CTRL_START_TRANS) {
		dev_dbg(&rsb->adapter.dev, "RSB bus busy\n");
		return -EBUSY;
	}

	reinit_completion(&rsb->complete);

	writel(RSB_INTS_LOAD_BSY | RSB_INTS_TRANS_ERR | RSB_INTS_TRANS_OVER,
	       rsb->regs + RSB_INTE);

	writel(RSB_CTRL_START_TRANS | RSB_CTRL_GLOBAL_INT_ENB,
	       rsb->regs + RSB_CTRL);

	wait_for_completion(&rsb->complete);

	if (rsb->status & RSB_INTS_LOAD_BSY) {
		dev_dbg(&rsb->adapter.dev, "RSB bus busy\n");
		return -EBUSY;
	}

	if (rsb->status & RSB_INTS_TRANS_ERR) {
		dev_dbg(&rsb->adapter.dev, "RSB bus xfer error\n");
		return -ENXIO;
	}

	return 0;
}

static int rsb_smbus_xfer(struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data)
{
	struct rsb *rsb = i2c_get_adapdata(adap);
	int ret;

	if (!data)
		return -EINVAL;

	writel(command, rsb->regs + RSB_ADDR);
	writel(RSB_DAR_RTA(addr), rsb->regs + RSB_DAR);

	if (read_write == I2C_SMBUS_READ) {
		if (size == I2C_SMBUS_BYTE_DATA)
			writel(RSB_CMD_RD8, rsb->regs + RSB_CMD);
		else
			writel(RSB_CMD_RD16, rsb->regs + RSB_CMD);
	} else {
		if (size == I2C_SMBUS_BYTE_DATA)
			writel(RSB_CMD_WR8, rsb->regs + RSB_CMD);
		else
			writel(RSB_CMD_WR16, rsb->regs + RSB_CMD);
	}

	ret = rsb_run_xfer(rsb);
	if (ret)
		return ret;

	if (read_write == I2C_SMBUS_READ) {
		if (size == I2C_SMBUS_BYTE_DATA)
			data->byte = readl(rsb->regs + RSB_DATA);
		else
			data->word = readl(rsb->regs + RSB_DATA);
	}

	return 0;
}

static u32 rsb_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA;
}

static const struct i2c_algorithm rsb_algo = {
	.smbus_xfer	= rsb_smbus_xfer,
	.functionality	= rsb_functionality,
};

/* following functions are used strictly during device probe */
static void rsb_init_device_mode(struct rsb *rsb)
{
	unsigned long expire = jiffies + msecs_to_jiffies(250);
	u32 reg;

	/* send init sequence */
	writel(RSB_DMCR_DEVICE_START | RSB_DMCR_MODE_DATA |
	       RSB_DMCR_MODE_REG | RSB_DMCR_DEV_ADDR, rsb->regs + RSB_DMCR);
	do {
		reg = readl(rsb->regs + RSB_DMCR);
	} while (time_before(jiffies, expire) && (reg & RSB_DMCR_DEVICE_START));

	if (reg & RSB_DMCR_DEVICE_START)
		dev_warn(&rsb->adapter.dev, "send init sequence timeout\n");

	/* clear interrupt status bits */
	writel(readl(rsb->regs + RSB_INTS), rsb->regs + RSB_INTS);
}

/* 15 valid runtime addresses for RSB slaves */
static const u8 rsb_valid_rtaddr[] = {
	0x17, 0x2d, 0x3a, 0x4e, 0x59, 0x63, 0x74, 0x8b,
	0x9c, 0xa6, 0xb1, 0xc5, 0xd2, 0xe8, 0xff,
};

static int rsb_check_rt_addr(u8 addr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rsb_valid_rtaddr); i++)
		if (addr == rsb_valid_rtaddr[i])
			return 0;

	return -EINVAL;
}

/* Scan the device tree for child nodes and set runtime address for them. */
static int rsb_set_rt_addrs(struct rsb *rsb)
{
	struct device *dev = rsb->adapter.dev.parent;
	struct device_node *child, *np = dev->of_node;
	u32 rt_addr, hw_addr;
	int ret;

	if (!np)
		return -EINVAL;

	for_each_child_of_node(np, child) {
		/* get hardware address */
		ret = of_property_read_u32(child, "allwinner,rsb-hw-addr",
					   &hw_addr);
		if (ret) {
			dev_warn(dev, "runtime address not given for %s\n",
				 of_node_full_name(child));
			continue;
		}

		/* get runtime address */
		ret = of_property_read_u32(child, "reg", &rt_addr);
		if (ret) {
			dev_warn(dev, "runtime address not given for %s\n",
				 of_node_full_name(child));
			continue;
		}

		/* check runtime address */
		ret = rsb_check_rt_addr(rt_addr);
		if (ret) {
			dev_warn(dev, "runtime address for %s is invalid\n",
				 of_node_full_name(child));
			continue;
		}

		/* setup command parameters */
		writel(RSB_CMD_STRA, rsb->regs + RSB_CMD);
		writel(RSB_DAR_RTA(rt_addr) | RSB_DAR_DA(hw_addr), rsb->regs + RSB_DAR);

		/* send command */
		ret = rsb_run_xfer(rsb);
		if (ret)
			dev_warn(dev, "set runtime address failed for %s\n",
				 of_node_full_name(child));
	}

	return 0;
}


static const struct of_device_id rsb_of_match_table[] = {
	{ .compatible = "allwinner,sun8i-a23-rsb" },
	{}
};
MODULE_DEVICE_TABLE(of, rsb_of_match_table);

static int rsb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	unsigned long parent_clk_freq;
	u32 clk_freq = 100000;
	struct resource *r;
	struct rsb *rsb;
	int clk_div;
	int irq;
	int ret;

	of_property_read_u32(np, "clock-frequency", &clk_freq);
	if (clk_freq > RSB_MAX_FREQ) {
		dev_err(dev,
			"clock-frequency (%u Hz) is too high (max = 20MHz)",
			clk_freq);
		return -EINVAL;
	}

	rsb = devm_kzalloc(dev, sizeof(struct rsb), GFP_KERNEL);
	if (!rsb)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rsb->regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(rsb->regs))
		return PTR_ERR(rsb->regs);

	strlcpy(rsb->adapter.name, pdev->name, sizeof(rsb->adapter.name));
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq: %d\n", irq);
		return irq;
	}

	rsb->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(rsb->clk)) {
		ret = PTR_ERR(rsb->clk);
		dev_err(dev, "failed to retrieve clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(rsb->clk);
	if (ret) {
		dev_err(dev, "failed to enable clk: %d\n", ret);
		return ret;
	}

	parent_clk_freq = clk_get_rate(rsb->clk);

	rsb->rstc = devm_reset_control_get(dev, NULL);
	if (IS_ERR(rsb->rstc)) {
		ret = PTR_ERR(rsb->rstc);
		dev_err(dev, "failed to retrieve reset controller: %d\n", ret);
		goto err_clk_disable;
	}

	ret = reset_control_deassert(rsb->rstc);
	if (ret) {
		dev_err(dev, "failed to deassert reset line: %d\n", ret);
		goto err_clk_disable;
	}

	init_completion(&rsb->complete);
	strlcpy(rsb->adapter.name, RSB_CTRL_NAME, sizeof(rsb->adapter.name));
	rsb->adapter.dev.parent = dev;
	rsb->adapter.algo = &rsb_algo;
	rsb->adapter.owner = THIS_MODULE;
	rsb->adapter.dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, rsb);
	i2c_set_adapdata(&rsb->adapter, rsb);

	writel(RSB_CTRL_SOFT_RST, rsb->regs + RSB_CTRL);

	clk_div = parent_clk_freq / clk_freq;
	if (!clk_div) {
		dev_warn(dev,
			 "clock-frequency is too high, setting it to %lu Hz\n",
			 parent_clk_freq);
		clk_div = 1;
	} else if (clk_div > RSB_CCR_MAX_CLK_DIV) {
		dev_warn(dev,
			 "clock-frequency is too low, setting it to %lu Hz\n",
			 parent_clk_freq / RSB_CCR_MAX_CLK_DIV);
		clk_div = RSB_CCR_MAX_CLK_DIV;
	}

	writel(RSB_CCR_SDA_OUT_DELAY(1) | RSB_CCR_CLK_DIV(clk_div),
	       rsb->regs + RSB_CCR);

	ret = devm_request_irq(dev, irq, rsb_interrupt, 0, RSB_CTRL_NAME, rsb);
	if (ret) {
		dev_err(dev, "can't register interrupt handler irq%d: %d\n",
			irq, ret);
		goto err_reset_assert;
	}

	rsb_init_device_mode(rsb);

	ret = rsb_set_rt_addrs(rsb);
	if (ret)
		goto err_reset_assert;

	ret = i2c_add_adapter(&rsb->adapter);
	if (!ret)
		return 0;

err_reset_assert:
	reset_control_assert(rsb->rstc);

err_clk_disable:
	clk_disable_unprepare(rsb->clk);

	return ret;
}

static int rsb_remove(struct platform_device *dev)
{
	struct rsb *rsb = platform_get_drvdata(dev);

	i2c_del_adapter(&rsb->adapter);
	reset_control_assert(rsb->rstc);
	clk_disable_unprepare(rsb->clk);

	return 0;
}

static struct platform_driver rsb_driver = {
	.probe = rsb_probe,
	.remove	= rsb_remove,
	.driver	= {
		.name = "i2c-sunxi-rsb",
		.of_match_table = rsb_of_match_table,
	},
};
module_platform_driver(rsb_driver);

MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_DESCRIPTION("Allwinner RSB driver");
MODULE_LICENSE("GPL v2");
