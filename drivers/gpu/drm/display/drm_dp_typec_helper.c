// SPDX-License-Identifier: GPL-2.0

#include <linux/usb/typec_mux.h>
#include <drm/display/drm_dp_helper.h>

static int drm_dp_register_mode_switch(struct device *dev,
				       struct fwnode_handle *fwnode,
				       struct drm_dp_typec_switch_desc *switch_desc,
				       void *data, typec_mux_set_fn_t mux_set)
{
	struct drm_dp_typec_port_data *port_data;
	struct typec_mux_desc mux_desc = {};
	char name[32];
	u32 port_num;
	int ret;

	ret = fwnode_property_read_u32(fwnode, "reg", &port_num);
	if (ret) {
		dev_err(dev, "Failed to read reg property: %d\n", ret);
		return ret;
	}

	port_data = &switch_desc->typec_ports[port_num];
	port_data->data = data;
	port_data->port_num = port_num;
	port_data->fwnode = fwnode;
	mux_desc.fwnode = fwnode;
	mux_desc.drvdata = port_data;
	snprintf(name, sizeof(name), "%pfwP-%u", fwnode, port_num);
	mux_desc.name = name;
	mux_desc.set = mux_set;

	port_data->typec_mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(port_data->typec_mux)) {
		ret = PTR_ERR(port_data->typec_mux);
		dev_err(dev, "Mode switch register for port %d failed: %d\n",
			port_num, ret);
		return ret;
	}

	return 0;
}

/**
 * drm_dp_register_typec_switches() - register Type-C switches
 * @dev: Device that registers Type-C switches
 * @port: Device node for the switch
 * @switch_desc: A Type-C switch descriptor
 * @data: Private data for the switches
 * @mux_set: Callback function for typec_mux_set
 *
 * This function registers USB Type-C switches for DP bridges that can switch
 * the output signal between their output pins.
 *
 * Currently only mode switches are implemented, and the function assumes the
 * given @port device node has endpoints with "mode-switch" property.
 * The port number is determined by the "reg" property of the endpoint.
 */
int drm_dp_register_typec_switches(struct device *dev, struct fwnode_handle *port,
				   struct drm_dp_typec_switch_desc *switch_desc,
				   void *data, typec_mux_set_fn_t mux_set)
{
	struct fwnode_handle *sw;
	int ret;

	fwnode_for_each_typec_mode_switch(port, sw)
		switch_desc->num_typec_switches++;

	if (!switch_desc->num_typec_switches) {
		dev_dbg(dev, "No Type-C switches node found\n");
		return 0;
	}

	switch_desc->typec_ports = devm_kcalloc(
		dev, switch_desc->num_typec_switches,
		sizeof(struct drm_dp_typec_port_data), GFP_KERNEL);

	if (!switch_desc->typec_ports)
		return -ENOMEM;

	/* Register switches for each connector. */
	fwnode_for_each_typec_mode_switch(port, sw) {
		ret = drm_dp_register_mode_switch(dev, sw, switch_desc, data, mux_set);
		if (ret)
			goto err_unregister_typec_switches;
	}

	return 0;

err_unregister_typec_switches:
	fwnode_handle_put(sw);
	drm_dp_unregister_typec_switches(switch_desc);
	return ret;
}
EXPORT_SYMBOL(drm_dp_register_typec_switches);

/**
 * drm_dp_unregister_typec_switches() - unregister Type-C switches
 * @switch_desc: A Type-C switch descriptor
 */
void drm_dp_unregister_typec_switches(struct drm_dp_typec_switch_desc *switch_desc)
{
	int i;

	for (i = 0; i < switch_desc->num_typec_switches; i++)
		typec_mux_unregister(switch_desc->typec_ports[i].typec_mux);
}
EXPORT_SYMBOL(drm_dp_unregister_typec_switches);
