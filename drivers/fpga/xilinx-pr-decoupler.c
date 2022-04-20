// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017, National Instruments Corp.
 * Copyright (c) 2017, Xilix Inc
 *
 * FPGA Bridge Driver for the Xilinx LogiCORE Partial Reconfiguration
 * Decoupler IP Core.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/fpga/fpga-bridge.h>

//PR decoupler enable/disable
#define CTRL_CMD_DECOUPLE	BIT(0)
#define CTRL_CMD_COUPLE		0
#define CTRL_OFFSET		0

//AXI firewall unblock
#define FIREWALL_SI_UNBLOCK_OFFSET 0x108
#define FIREWALL_SI_CMD_UNBLOCK    BIT(0)

#define FIREWALL_SI_FAULT_STATUS   0x104
#define WR_DECERR                  BIT(26)
#define WR_SLVERR                  BIT(25)
#define ERRM_WVALID_STABLE         BIT(24)
#define ERRM_AWVALID_STABLE        BIT(23)
#define ERRM_AWADDR_BOUNDARY       BIT(22)
#define ERRM_WDATA_NUM             BIT(21)
#define ERRM_AWSIZE                BIT(20)
#define RECM_W_TO_AW_VALID_TIMEOUT BIT(19)
#define RECM_WTRANS_TIMEOUT        BIT(18)
#define RECM_BREADY_TIMEOUT        BIT(17)
#define SI_FAULT_WR_STATUS_MASK    BIT(27) - BIT(17)

#define RD_DECERR                  BIT(6)
#define RD_SLVERR                  BIT(5)
#define ERRM_ARVALID_STABLE        BIT(4)
#define ERRM_ARADDR_BOUNDARY       BIT(3)
#define ERRM_ARSIZE                BIT(2)
#define RECM_RREADY_TIMEOUT        BIT(1)
#define SI_FAULT_RD_STATUS_MASK    BIT(7) - BIT(1)

#define WR_RESP_BUSY        BIT(16)
#define RD_RESP_BUSY        BIT(0)

struct xlnx_pr_decoupler_data {
	void __iomem *io_base_bridge;
	void __iomem *io_base_firewall;
	struct clk *clk;
};

static inline void xlnx_axi_firewall_write(struct xlnx_pr_decoupler_data *d, 
		                             u32 offset, u32 val)
{
	writel(val, d->io_base_firewall + offset);
}

static inline u32 xlnx_axi_firewall_read(struct xlnx_pr_decoupler_data *d, 
		                         u32 offset)
{
	return readl(d->io_base_firewall + offset);
}

static inline void xlnx_pr_decoupler_write(struct xlnx_pr_decoupler_data *d,
					   u32 offset, u32 val)
{
	writel(val, d->io_base_bridge + offset);
}

static inline u32 xlnx_pr_decoupler_read(const struct xlnx_pr_decoupler_data *d,
					u32 offset)
{
	return readl(d->io_base_bridge + offset);
}

static int xlnx_pr_decoupler_enable_set(struct fpga_bridge *bridge, bool enable)
{
	int err;
	struct xlnx_pr_decoupler_data *priv = bridge->priv;

	err = clk_enable(priv->clk);
	if (err)
		return err;

	if (enable)
		//PR coupling (release ROLE resetn)
		xlnx_pr_decoupler_write(priv, CTRL_OFFSET, CTRL_CMD_COUPLE);
	else
		xlnx_pr_decoupler_write(priv, CTRL_OFFSET, CTRL_CMD_DECOUPLE);

	clk_disable(priv->clk);

	return 0;
}

static int xlnx_pr_decoupler_enable_show(struct fpga_bridge *bridge)
{
	const struct xlnx_pr_decoupler_data *priv = bridge->priv;
	u32 status;
	int err;

	err = clk_enable(priv->clk);
	if (err)
		return err;

	status = readl(priv->io_base_bridge);

	clk_disable(priv->clk);

	return !status;
}

static const struct fpga_bridge_ops xlnx_pr_decoupler_br_ops = {
	.enable_set = xlnx_pr_decoupler_enable_set,
	.enable_show = xlnx_pr_decoupler_enable_show,
};

static const struct of_device_id xlnx_pr_decoupler_of_match[] = {
	{ .compatible = "xlnx,pr-decoupler-1.00", },
	{ .compatible = "xlnx,pr-decoupler", },
	{},
};
MODULE_DEVICE_TABLE(of, xlnx_pr_decoupler_of_match);

static int xlnx_pr_decoupler_probe(struct platform_device *pdev)
{
	struct xlnx_pr_decoupler_data *priv;
	struct fpga_bridge *br;
	int err;
	struct resource *res;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	//parse bridge (PR decoupler) MMIO base address
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bridge");
	priv->io_base_bridge = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->io_base_bridge))
		return PTR_ERR(priv->io_base_bridge);

	//parse bus firewall MMIO base address
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "firewall");
	priv->io_base_firewall = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->io_base_firewall))
		return PTR_ERR(priv->io_base_firewall);

	priv->clk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(priv->clk)) {
		if (PTR_ERR(priv->clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "input clock not found\n");
		return PTR_ERR(priv->clk);
	}

	err = clk_prepare_enable(priv->clk);
	if (err) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return err;
	}

	clk_disable(priv->clk);

	br = devm_fpga_bridge_create(&pdev->dev, "Xilinx PR Decoupler",
				     &xlnx_pr_decoupler_br_ops, priv);
	if (!br) {
		err = -ENOMEM;
		goto err_clk;
	}

	platform_set_drvdata(pdev, br);

	err = fpga_bridge_register(br);
	if (err) {
		dev_err(&pdev->dev, "unable to register Xilinx PR Decoupler");
		goto err_clk;
	}

	return 0;

err_clk:
	clk_unprepare(priv->clk);

	return err;
}

static int xlnx_pr_decoupler_remove(struct platform_device *pdev)
{
	struct fpga_bridge *bridge = platform_get_drvdata(pdev);
	struct xlnx_pr_decoupler_data *p = bridge->priv;

	fpga_bridge_unregister(bridge);

	clk_unprepare(p->clk);

	return 0;
}

static struct platform_driver xlnx_pr_decoupler_driver = {
	.probe = xlnx_pr_decoupler_probe,
	.remove = xlnx_pr_decoupler_remove,
	.driver = {
		.name = "xlnx_pr_decoupler",
		.of_match_table = of_match_ptr(xlnx_pr_decoupler_of_match),
	},
};

module_platform_driver(xlnx_pr_decoupler_driver);

MODULE_DESCRIPTION("Xilinx Partial Reconfiguration Decoupler");
MODULE_AUTHOR("Moritz Fischer <mdf@kernel.org>");
MODULE_AUTHOR("Michal Simek <michal.simek@xilinx.com>");
MODULE_LICENSE("GPL v2");
