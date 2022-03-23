/*
 *  Driver for SCM701D SCU controller
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <mach/hardware.h>
#include <mach/ftscu100.h>
#include <dt-bindings/scu/scu-leo.h>

#define SCM701D_SCU_MAGIC_NUM_SIZE 4

static int scm701d_scu_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	const __be32 *list = NULL;
	int size;
	int i;
	unsigned int value;
#if 1
	list = of_get_property(np, "pll,control", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		if (0 == (value & PLL1_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x8040);
		}
		if (0 == (value & PLL2_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x8048);
		}
		if (0 == (value & PLL3_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x8050);
		}
		if (0 == (value & PLL4_CONTROL_EN)) {
			/* cannot close !!! */
		}
		if (0 == (value & PLL5_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x8060);
		}
		if (0 == (value & PLL6_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x80C8);
		}
		if (0 == (value & PLL7_CONTROL_EN)) {
			writel(0, (void __iomem *)PLAT_SCU_VA_BASE + 0x8114);
		}
	}

	list = of_get_property(np, "AHB,control", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x50);
		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x58);
	}


	list = of_get_property(np, "APB0,control", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x60);
		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x68);
	}

	list = of_get_property(np, "APB1,control", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x64);
		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x6C);
	}

	list = of_get_property(np, "AXI,control", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x80);
		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x88);
	}

	list = of_get_property(np, "clock0,enable", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x8070);
	}

	list = of_get_property(np, "clock1,enable", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x8074);
	}

	list = of_get_property(np, "clock2,enable", &size);
	if (list){
		value = 0;
		size = size / SCM701D_SCU_MAGIC_NUM_SIZE;
		
		for (i = 0; i < size; i++) {
			value |= be32_to_cpu(*list++);
		}

		writel(value, (void __iomem *)PLAT_SCU_VA_BASE + 0x8078);
	}
#endif
	return 0;
}

static int scm701d_scu_remove(struct platform_device *pdev)
{
	return 0;
}


static struct of_device_id scm710d_scu_of_match[] = {
	{ .compatible = "scm701d-scu"},
	{ },
};

static struct platform_driver scm701d_scu_driver = {
	.driver = {
		.name = "scm701d_scu",
		.owner = THIS_MODULE,
		.of_match_table = scm710d_scu_of_match,
	},
	.remove = scm701d_scu_remove,
};

static int __init scm701d_scu_init(void)
{
	return platform_driver_probe(&scm701d_scu_driver, scm701d_scu_probe);
}
arch_initcall(scm701d_scu_init);

static void __exit scm701d_scu_exit(void)
{
	platform_driver_unregister(&scm701d_scu_driver);
}
module_exit(scm701d_scu_exit);

MODULE_AUTHOR("hechaohai<hech@vangotech.com");
MODULE_DESCRIPTION("SCM701D scu driver");
MODULE_LICENSE("GPL");

