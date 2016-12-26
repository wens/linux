/*
 * Copyright (c) 2016 Chen-Yu Tsai
 *
 * Chen-Yu Tsai <wens@csie.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/bridge/dw_hdmi.h>

#include "sun4i_drv.h"
#include "sun4i_tcon.h"

struct dw_hdmi_sunxi {
	struct drm_encoder encoder;
	struct device *dev;
	struct clk *clk_mod;
	struct reset_control *rstc_dw_hdmi;
	struct reset_control *rstc_wrapper;
	struct sun4i_drv *drv;
};

#define to_dw_hdmi_sunxi(x)	container_of(x, struct dw_hdmi_sunxi, x)

/* TODO The vendor does not support 10/12 bpp modes. No values were provided */
static const struct dw_hdmi_mpll_config sunxi_mpll_cfg[] = {
	{
		27000000, {
			{ 0x01e0, 0x0000 },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 }
		},
	}, {
		74250000, {
			{ 0x0540, 0x0005 },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 }
		},
	}, {
		148500000, {
			{ 0x04a0, 0x000a },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 }
		},
	}, {
		297000000, {
			{ 0x0000, 0x000f },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 }
		},
	}, {
		~0UL, {
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
			{ 0x0000, 0x0000 },
		},
	}
};

static const struct dw_hdmi_curr_ctrl sunxi_cur_ctr[] = {
	/* pixelclk    bpp8    bpp10   bpp12 */
	{  27000000, { 0x08da, 0x0000, 0x0000 } },
	{  74250000, { 0x0000, 0x0000, 0x0000 } },
	{ 148500000, { 0x0000, 0x0000, 0x0000 } },
	{ 297000000, { 0x0000, 0x0000, 0x0000 } },
	{      ~0UL, { 0x0000, 0x0000, 0x0000 } }
};

static const struct dw_hdmi_phy_config sunxi_phy_config[] = {
	/*pixelclk   symbol   term   vlev*/
	{  27000000, 0x8009, 0x0007, 0x0318 },
	{  74250000, 0x8009, 0x0007, 0x02b5 },
	{ 148500000, 0x8029, 0x0002, 0x0021 },
	{ 297000000, 0x802b, 0x0002, 0x0000 },
	{      ~0UL, 0x0000, 0x0000, 0x0000 }
};

static enum drm_mode_status
dw_hdmi_sunxi_mode_valid(struct drm_connector *connector,
			    struct drm_display_mode *mode)
{
	const struct dw_hdmi_mpll_config *mpll_cfg = sunxi_mpll_cfg;
	int pclk = mode->clock * 1000;
	bool valid = false;
	int i;

	for (i = 0; mpll_cfg[i].mpixelclock != (~0UL); i++) {
		if (pclk == mpll_cfg[i].mpixelclock) {
			valid = true;
			break;
		}
	}

	return (valid) ? MODE_OK : MODE_BAD;
}

static void dw_hdmi_sunxi_disable(struct drm_encoder *encoder)
{
	struct dw_hdmi_sunxi *hdmi = to_dw_hdmi_sunxi(encoder);
	struct sun4i_drv *drv = hdmi->drv;
	struct sun4i_tcon *tcon = drv->tcon;

	DRM_DEBUG_DRIVER("Disabling HDMI Output\n");

	sun4i_tcon_channel_disable(tcon, 1);
	clk_disable_unprepare(hdmi->clk_mod);
}

static void dw_hdmi_sunxi_enable(struct drm_encoder *encoder)
{
	struct dw_hdmi_sunxi *hdmi = to_dw_hdmi_sunxi(encoder);
	struct sun4i_drv *drv = hdmi->drv;
	struct sun4i_tcon *tcon = drv->tcon;

	DRM_DEBUG_DRIVER("Enabling HDMI Output\n");

	clk_prepare_enable(hdmi->clk_mod);
	sun4i_tcon_channel_enable(tcon, 1);
}

static void dw_hdmi_sunxi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct dw_hdmi_sunxi *hdmi = to_dw_hdmi_sunxi(encoder);
	struct sun4i_drv *drv = hdmi->drv;
	struct sun4i_tcon *tcon = drv->tcon;

	sun4i_tcon1_mode_set(tcon, mode);

	clk_set_rate(tcon->sclk1, mode->crtc_clock * 1000);
	clk_set_rate(hdmi->clk_mod, mode->crtc_clock * 1000);
}

static const struct drm_encoder_helper_funcs dw_hdmi_sunxi_encoder_helper_funcs = {
	.disable  = dw_hdmi_sunxi_disable,
	.enable   = dw_hdmi_sunxi_enable,
	.mode_set = dw_hdmi_sunxi_mode_set,
};

static const struct drm_encoder_funcs dw_hdmi_sunxi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct dw_hdmi_plat_data dw_hdmi_sunxi_drv_data = {
	.mode_valid = dw_hdmi_sunxi_mode_valid,
	.mpll_cfg   = sunxi_mpll_cfg,
	.cur_ctr    = sunxi_cur_ctr,
	.phy_config = sunxi_phy_config,
	.dev_type   = A80_HDMI,
};

static int dw_hdmi_sunxi_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct sun4i_drv *drv = drm->dev_private;
	struct drm_encoder *encoder;
	struct dw_hdmi_sunxi *hdmi;
	struct resource *iores;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iores)
		return -ENXIO;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->drv = drv;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	/* TODO Use drm_of_find_possible_crtcs */
	encoder->possible_crtcs = BIT(0);

	hdmi->clk_mod = devm_clk_get(dev, "imod");
	if (IS_ERR(hdmi->clk_mod)) {
		dev_err(dev, "Could not get module clock\n");
		return PTR_ERR(hdmi->clk_mod);
	}

	hdmi->rstc_wrapper = devm_reset_control_get(dev, "wrapper");
	if (IS_ERR(hdmi->rstc_wrapper)) {
		dev_err(dev, "Could not get wrapper reset control\n");
		return PTR_ERR(hdmi->rstc_wrapper);
	}

	hdmi->rstc_dw_hdmi = devm_reset_control_get(dev, "dw-hdmi");
	if (IS_ERR(hdmi->rstc_dw_hdmi)) {
		dev_err(dev, "Could not get dw-hdmi reset control\n");
		return PTR_ERR(hdmi->rstc_dw_hdmi);
	}

	ret = reset_control_deassert(hdmi->rstc_wrapper);
	if (ret) {
		dev_err(dev, "Could not deassert wrapper reset control\n");
		return ret;
	}

	ret = reset_control_deassert(hdmi->rstc_dw_hdmi);
	if (ret) {
		dev_err(dev, "Could not deassert dw-hdmi reset control\n");
		goto err_assert_wrapper_reset;
	}

	drm_encoder_helper_add(encoder, &dw_hdmi_sunxi_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &dw_hdmi_sunxi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	ret = dw_hdmi_bind(dev, master, data, encoder, iores, irq, &dw_hdmi_sunxi_drv_data);
	if (ret)
		goto err_cleanup_encoder;

	return 0;

err_cleanup_encoder:
	drm_encoder_cleanup(encoder);
	reset_control_assert(hdmi->rstc_dw_hdmi);
err_assert_wrapper_reset:
	reset_control_assert(hdmi->rstc_wrapper);

	return ret;
}

static void dw_hdmi_sunxi_unbind(struct device *dev, struct device *master,
				 void *data)
{
	dw_hdmi_unbind(dev, master, data);
}

static const struct component_ops dw_hdmi_sunxi_ops = {
	.bind	= dw_hdmi_sunxi_bind,
	.unbind	= dw_hdmi_sunxi_unbind,
};

static int dw_hdmi_sunxi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dw_hdmi_sunxi_ops);
}

static int dw_hdmi_sunxi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dw_hdmi_sunxi_ops);

	return 0;
}

static const struct of_device_id dw_hdmi_sunxi_dt_ids[] = {
	{ .compatible = "allwinner,sun9i-a80-hdmi" },
	{},
};
MODULE_DEVICE_TABLE(of, dw_hdmi_sunxi_dt_ids);

static struct platform_driver dw_hdmi_sunxi_pltfm_driver = {
	.probe  = dw_hdmi_sunxi_probe,
	.remove = dw_hdmi_sunxi_remove,
	.driver = {
		.name = "dw-hdmi-sunxi",
		.of_match_table = dw_hdmi_sunxi_dt_ids,
	},
};

module_platform_driver(dw_hdmi_sunxi_pltfm_driver);

MODULE_AUTHOR("Chen-Yu Tsai <wens@csie.org>");
MODULE_DESCRIPTION("Allwinner Specific DW-HDMI Driver Extension");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dw-hdmi-sunxi");
