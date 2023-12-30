#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy/pcie.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

static int phytest_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy *phy;
	int ret;

	phy = devm_phy_get(dev, NULL);
	if (IS_ERR(phy))
		return -1;

	ret = phy_init(phy);
	if (ret)
		return -2;

	ret = phy_set_mode_ext(phy, PHY_MODE_PCIE, PHY_MODE_PCIE_RC);
	if (ret)
		return -3;

	ret = phy_power_on(phy);
	if (ret)
		return -4;

	return 0;
};

static const struct of_device_id phytest_match_table[] = {
	{ .compatible = "phytest" },
	{ }
};
MODULE_DEVICE_TABLE(of, phytest_match_table);

static struct platform_driver phytest_driver = {
	.probe = phytest_probe,
	.driver = {
		.name = "phy-test",
		.of_match_table = phytest_match_table,
	},
};
module_platform_driver(phytest_driver);

MODULE_LICENSE("GPL");
