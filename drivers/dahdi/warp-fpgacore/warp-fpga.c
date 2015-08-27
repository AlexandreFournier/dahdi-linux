/**
 * WARP-FPGA
 *
 * - This driver is for Warp V2 using PLB/OPB platform device
 * - This driver is for Warp V3 using PCIe device
 */

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <pika/driver.h>
#include <pika/warp-fpga.h>
#include <pika/pksystemver.h>

#ifdef WARP_V2
#include <linux/of_address.h>
#include <linux/of_irq.h>
#else
#include <linux/pci.h>
#endif

//#define WARP_FPGA_DEBUG_IO 1

#define DRIVER_NAME "warp_fpga"

int debug = 0;

module_param(debug, int, 0664);
#define TRIGGER_RECONFIG_FLAG 0x08000000

/************************************************************/
/* Global Veriables                                         */
/************************************************************/
static PDEVICE_EXTENSION pdx;
static spinlock_t fpgalock;
static int warpfpga_active = 0;

#ifdef WARP_V2
static struct platform_device * g_warp_fpga_dev = NULL;
#else
static struct pci_dev * g_warp_fpga_dev = NULL;
#endif

/************************************************************/
/* Prototypes                                               */
/************************************************************/
int taco_proc_init(PDEVICE_EXTENSION pdx);
int taco_proc_remove(void);

int warpfpgactrl_init(void);
int warpfpgactrl_remove(void);

void warp_fpga_reconfig(void);

#ifdef WARP_V2
/************************************************************/
/* Platform-Device Functions                               */
/************************************************************/
static int warp_fpga_probe(struct platform_device *pdev)
{
	struct warp_fpga *chip = NULL;
	struct device_node *np;
	int tries, ret = 0;
	struct resource res;

	// Create the Warp FPGA structure
	printk(KERN_INFO "Allocating FPGA structure\n");
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;
	chip->pdev = pdev;
	chip->dev = &pdev->dev;

	// Extract all FPGA parameters from open firmware
	np = of_find_compatible_node(NULL, NULL, "pika,fpga");
	if (np == NULL) {
		dev_err(&pdev->dev, "%s of_find_compatible_node FAILED\n", __func__);
		goto error_cleanup;
	}

	// Check resource
	if (of_address_to_resource(np, 0, &res)) {
		dev_err(&pdev->dev, "%s of_address_to_resource FAILED\n", __func__);
		goto error_cleanup;
	}
	printk(KERN_INFO "pika,fpga resource => %x-%x\n", res.start, res.end);

	// Map FPGA base
	chip->base = chip->fpga = ioremap(res.start, 0x2200);
	if (chip->fpga == NULL) {
		dev_err(&pdev->dev, "%s ioremap FAILED\n", __func__);
		goto error_cleanup;
	}

	// Setup the IRQ
	chip->irq = irq_of_parse_and_map(np, 0);
	if (chip->irq == NO_IRQ) {
		dev_err(&pdev->dev, "%s irq_of_parse_and_map FAILED\n", __func__);
		goto error_cleanup;
	}

	// Done with np
    of_node_put(np);

	// Extract all FPGA-SGL parameters from open firmware
	np = of_find_compatible_node(NULL, NULL, "pika,fpga-sgl");

	// Map FPGA-SGL base
	chip->sgl = of_iomap(np, 0);
	if (chip->sgl == NULL) {
		dev_err(&pdev->dev, "%s of_iomap FAILED\n", __func__);
		goto error_cleanup;
	}

	// Done with np
    of_node_put(np);

	// Mutex creation
	mutex_init(&chip->lock);

	// Backup platform device in driver data
	platform_set_drvdata(pdev, chip);

	// Int the old pdx struct to be compatiable with the read of function
	pdx->info.irql = chip->irq;
	pdx->pdev = chip->pdev;
	pdx->bar0 = chip->fpga;

	printk(KERN_INFO "PDX OK (irq:%d) (bar0:%p)\n", pdx->info.irql, pdx->bar0);

	// Verify current FPGA mode of operation
	for (tries = 0; tries < 3; tries++)
	{
		if ((fpga_read(chip->base, BAR0_FPGA_UPDATE) & 0x300) == 0x0)
		{
			dev_err(&pdev->dev, "Detected FPGA in Factory mode\n");

			// Detected Factory Mode, try to restart the ?
			warp_fpga_reconfig();

			if ((fpga_read(chip->base, BAR0_FPGA_UPDATE) & 0x300) == 0x100)
			{
				dev_info(&pdev->dev, "FPGA restored mode to operational\n");
				break;
			}
			continue;
		}
		break;
	}

	// Initialize the procfs interface
	taco_proc_init(pdx);

  	// Init the DMA buffers and scatter-gather list
	ret = dma_init_module(chip);
	if (ret != 0) {
		dev_err(&pdev->dev, "dma_init_module FAILED-%d", ret);
		goto err_taco;
	}

	if (warpfpgactrl_init() != 0)
	{
		dev_err(&pdev->dev, "warpfpgactrl_init FAILED");
		goto err_taco;

	}

	// Only reset the silabs once
	/*
	if (daytona_silabs_reset(pdx)) {
		printk(KERN_ERR "Unable to reset silabs\n");
		goto err_taco;
	}
	*/

	printk("Base FPGA Address: %p = %p\n", chip->fpga, chip->base);

	// Set the device
	g_warp_fpga_dev = pdev;
	warpfpga_active = 1;

#ifndef WARP_V2
	// Place DSP in Bypass mode if DSP count is zero
	if (warp_fpga_dsp_count() == 0)
	{
		printk("Placing the FPGA in DSP Bypass Mode since no DSP found\n");
		u32 val = fpga_read(chip->base, FPGA_DIAG) | 0x800;
		fpga_write(chip->base, FPGA_DIAG, val);
	}
#endif

	return ret;

err_taco:
	taco_proc_remove();

error_cleanup:
	if (np)
		of_node_put(np);
	if (chip->fpga)
		iounmap(chip->fpga);
	if (chip->sgl)
		iounmap(chip->sgl);

	kfree(chip);
	dev_err(&pdev->dev, "%s Failed returns %d\n", __func__, ret);
	return ret;
}

static int warp_fpga_remove(struct platform_device *pdev)
{
	struct warp_fpga *chip = warp_fpga_getdevice();

	disable_irq(chip->irq);

	fpga_write(pdx->bar0, FPGA_CONFIG, fpga_read(pdx->bar0, FPGA_CONFIG) | TRIGGER_RECONFIG_FLAG );
	udelay(100);
	fpga_write(pdx->bar0, FPGA_CONFIG, fpga_read(pdx->bar0, FPGA_CONFIG) & ~ TRIGGER_RECONFIG_FLAG );
	ssleep(5);

	enable_irq(chip->irq);

	free_irq(chip->irq, chip);
	chip->sgl = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int warp_fpga_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int warp_fpga_resume(struct platform_device *pdev)
{
    return 0;
}
#endif

/************************************************************/
/* Plateform-Device definitions                             */
/************************************************************/
static const struct of_device_id warp_fpga_of_device_id[] = {
	{ .compatible = "pika,fpga" },
	{ }
};
static struct platform_driver warp_fpga_platform_driver = {
	.driver         = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = warp_fpga_of_device_id
	},
	.probe          = warp_fpga_probe,
	.remove         = warp_fpga_remove,
#ifdef	CONFIG_PM
	.suspend = warp_fpga_suspend,
	.resume = warp_fpga_resume
#endif
};

#else
/************************************************************/
/* PCI Functions                                            */
/************************************************************/

static int warp_fpga_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	s32 ret = 0;
	struct warp_fpga *chip = NULL;
	int tries;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->pdev = pdev;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s : pci_enable_device FAILED", __func__);
		goto err_pci_enable;
	}

	// Set the PCI Master Bit to true always in the PCI bridge
	pci_set_master(pdev);

	dev_dbg(&pdev->dev, "try set_consistent_dma_mask(32)\n");
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "set_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}
	
	ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "set_consistent_dma_mask(32) failed\n");
		goto err_set_dma_mask;
	}

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret) {
		dev_err(&pdev->dev, "pci_request_regions FAILED-%d", ret);
		goto err_request_regions;
	}

	chip->base = pci_iomap(pdev, 0, 0);
	if (chip->base == 0) {
		dev_err(&pdev->dev, "%s : pci_iomap FAILED", __func__);
		ret = -ENOMEM;
		goto err_iomap;
	}

	pci_set_drvdata(pdev, chip);
	mutex_init(&chip->lock);

	// Setup the IRQ
	chip->irq = pdev->irq;
	chip->fpga = chip->base;

	// Int the old pdx struct to be compatiable with the read of function
	pdx->info.irql = chip->irq;
	pdx->bar0 = chip->base;
	pdx->pdev = pdev;

	// Verify current FPGA mode of operation
	for (tries = 0; tries < 2; tries++)
	{
		if ((fpga_read(chip->base, BAR0_FPGA_UPDATE) & 0x300) == 0x0)
		{
			dev_err(&pdev->dev, "Detected FPGA in Factory mode\n");

			// Detected Factory Mode, try to restart the pci_bus
			warp_fpga_reconfig();

			if ((fpga_read(chip->base, BAR0_FPGA_UPDATE) & 0x300) == 0x100)
			{
				dev_info(&pdev->dev, "FPGA restored mode to operational\n");
				break;
			}
			continue;
		}
		break;
	}

	// Initialize the procfs interface
	taco_proc_init(pdx);

  	// Init the dma buffer and scatter list
	ret = dma_init_module(chip);
	if (ret != 0) {
		dev_err(&pdev->dev, "dma_init_module FAILED-%d", ret);
		goto err_taco;
	}

	if (warpfpgactrl_init(chip) != 0 )
	{
		dev_err(&pdev->dev, "warpfpgactrl_init FAILED");
		goto err_taco;

	}

	// Only reset the silabs once
	if (daytona_silabs_reset(pdx)) {
		printk(KERN_ERR "Unable to reset silabs\n");
		goto err_taco;
	}

	printk("Base FPGA Address: %p = %p\n", chip->fpga, chip->base);

	// Set the device
	g_warp_fpga_dev = pdev;
	warpfpga_active = 1;

	// Place DSP in Bypass mode if DSP count is zero
	if (warp_fpga_dsp_count() == 0)
	{
		printk("Placing the FPGA in DSP Bypass Mode since no DSP found\n");
		u32 val = fpga_read(chip->base, BAR0_DIAG) | 0x800;
		fpga_write(chip->base, BAR0_DIAG,val);
	}

	return ret;

err_taco:
	taco_proc_remove();

err_iomap:
	pci_release_regions(pdev);

err_set_dma_mask:
err_request_regions:
	pci_disable_device(pdev);

err_pci_enable:
	kfree(chip);
	dev_err(&pdev->dev, "%s Failed returns %d\n", __func__, ret);
	return ret;
}

static void warp_fpga_remove(struct pci_dev *pdev)
{
	struct warp_fpga *chip = pci_get_drvdata(pdev);
	dma_exit_module();
	pci_iounmap(pdev, chip->base);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	warpfpgactrl_remove();
	taco_proc_remove();
	kfree(chip);
	g_warp_fpga_dev = NULL;
	warpfpga_active = 0;
}

#ifdef CONFIG_PM
static int warp_fpga_suspend(struct pci_dev *pdev, pm_message_t state)
{
    return 0;
}

static int warp_fpga_resume(struct pci_dev *pdev)
{
    return 0;
}
#endif

/************************************************************/
/* PCI definitions                                          */
/************************************************************/
//{ 0xXXXX, 0xXXX, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 }
static DEFINE_PCI_DEVICE_TABLE(warp_fpga_pcidev_id) = { 
		{ PCI_DEVICE(0x16DF, 0x4C10) },
		{ 0, }
};

MODULE_DEVICE_TABLE(pci, warp_fpga_pcidev_id);

static struct pci_driver warp_fpga_pci_driver = { 
	.name = DRIVER_NAME,
	.id_table = warp_fpga_pcidev_id,
	.probe = warp_fpga_probe,
	.remove = warp_fpga_remove,
#ifdef	CONFIG_PM
	.suspend = warp_fpga_suspend,
	.resume = warp_fpga_resume
#endif
};
#endif // WARP_V2

/************************************************************/
/* Functions                                                */
/************************************************************/

struct warp_fpga * warp_fpga_getdevice(void)
{
	if (g_warp_fpga_dev)
		return platform_get_drvdata(g_warp_fpga_dev);

	return NULL;
}
EXPORT_SYMBOL(warp_fpga_getdevice);

void warp_fpga_reconfig(void)
{
	struct warp_fpga *chip = warp_fpga_getdevice();

	disable_irq(chip->irq);

#ifndef WARP_V2
	pci_save_state(chip->pdev);
	pci_disable_device(chip->pdev);
#endif

	fpga_write(chip->fpga, FPGA_CONFIG, fpga_read(chip->fpga, FPGA_CONFIG) | TRIGGER_RECONFIG_FLAG );
	udelay(100);
	fpga_write(chip->fpga, FPGA_CONFIG, fpga_read(chip->fpga, FPGA_CONFIG) & ~ TRIGGER_RECONFIG_FLAG );
	ssleep(5);

#ifndef WARP_V2
	pci_restore_state(chip->pdev);
	pci_enable_device(chip->pdev);
#endif

	enable_irq(chip->irq);
}
EXPORT_SYMBOL(warp_fpga_reconfig);

void warp_fpga_enable_irq(int irq)
{
	//imr_set(pdx, irq);	
	unsigned imr = fpga_read(pdx->bar0, BAR0_IMR);
//	irq &= ~IMR_MASK;
	fpga_write(pdx->bar0, BAR0_IMR, imr | irq);
	pdx->imr |= irq;
	if (debug)
		printk("%s: irq: %x setting fpga imr to: %08x\n", __FUNCTION__, irq, imr | irq );
}
EXPORT_SYMBOL(warp_fpga_enable_irq);

void warp_fpga_disable_irq(int irq)
{
	unsigned imr = fpga_read(pdx->bar0, BAR0_IMR);
//	irq &= ~IMR_MASK;
	BAR0_WRITE(pdx, BAR0_IMR, imr & ~irq);
	pdx->imr &= ~irq;
	if (debug)
		printk("%s: irq: %x setting fpga imr to: %08x\n", __FUNCTION__, irq, imr & ~ irq );
}
EXPORT_SYMBOL(warp_fpga_disable_irq);

int warp_fpga_isactive(void)
{
	return warpfpga_active;
}
EXPORT_SYMBOL(warp_fpga_isactive);

int warp_fpga_dsp_count(void)
{
	u32 config_reg;
	int count = 0;

	/* Detect number of DSP installed */
	config_reg = fpga_read(pdx->bar0, FPGA_CONFIG) & 0x3000000;
	config_reg >>= 24;
	if (config_reg > 0 )
	{	
		if (( config_reg & 0x2 ) == 0x2 ) 
			count++;

		if (( config_reg & 0x1 ) == 0x1 ) 
			count++;

	}

	return count;
}
EXPORT_SYMBOL(warp_fpga_dsp_count);

u32 fpga_read(void __iomem *fpga, int reg)
{
	u32 value = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&fpgalock, flags);
#ifdef WARP_V2
	value = in_be32(fpga + reg);
#else
	value = readl(fpga + reg);
#endif
	spin_unlock_irqrestore(&fpgalock, flags);

#ifdef WARP_FPGA_DEBUG_IO
	printk(KERN_INFO KBUILD_MODNAME ": R32(0x%p, 0x%04x) => %08x\n", fpga, reg, value);
#endif
	return value;
}
EXPORT_SYMBOL(fpga_read);

void fpga_write(void __iomem *fpga, int reg, u32 value)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&fpgalock, flags);
#ifdef WARP_V2
	out_be32(fpga + reg, value);
#else
	writel(value, fpga + reg);
#endif
	spin_unlock_irqrestore(&fpgalock, flags);

#ifdef WARP_FPGA_DEBUG_IO
	printk(KERN_INFO KBUILD_MODNAME ": W32(0x%p, 0x%04x, %08x)\n", fpga, reg, value);
#endif
}
EXPORT_SYMBOL(fpga_write);

u16 fpga_readw(void __iomem *fpga, int reg)
{
	u16 value = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&fpgalock, flags);
#ifdef WARP_V2
	value = in_be16(fpga + reg);
#else
	value = readw(fpga + reg);
#endif
	spin_unlock_irqrestore(&fpgalock, flags);

#ifdef WARP_FPGA_DEBUG_IO
	printk(KERN_INFO KBUILD_MODNAME ": R16(0x%p, 0x%04x) => %04x\n", fpga, reg, value);
#endif
	return value;
}
EXPORT_SYMBOL(fpga_readw);

void fpga_writew(void __iomem *fpga, int reg, u16 value)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&fpgalock, flags);
#ifdef WARP_V2
	out_be16(fpga + reg, value);
#else
	writew(value, fpga+reg);
#endif
	spin_unlock_irqrestore(&fpgalock, flags);

#ifdef WARP_FPGA_DEBUG_IO
	printk(KERN_INFO KBUILD_MODNAME ": W16(0x%p, 0x%04x, %08x)\n", fpga, reg, value);
#endif
}
EXPORT_SYMBOL(fpga_writew);

/************************************************************/
/* Module Init and Cleanup                                  */
/************************************************************/
static int __init warpfpga_init_module(void)
{
	int ret = 0;

	// Create PDX global veriable - this is for legacy uses
	if ((pdx = kzalloc(sizeof(DEVICE_EXTENSION), GFP_KERNEL)) == NULL) {
		printk(KERN_ERR "%s:%s() error allocating memory\n",
			__FILE__, __FUNCTION__);
		return -ENOMEM;
	}

	// You need a device for DMA or nothing works with 2.6.31
	pdx->dev = platform_device_register_simple("warp-dev", 0, NULL, 0);
	pdx->dev->dev.coherent_dma_mask = ~0ULL;

	// Create spin-lock
	spin_lock_init(&fpgalock);

	// Register device
	printk(KERN_INFO "Registering WARP FPGA driver\n");
#ifdef WARP_V2
	ret = platform_driver_register(&warp_fpga_platform_driver);
#else
	ret = pci_register_driver(&warp_fpga_pci_driver);
#endif
	printk(KERN_INFO "WARP FPGA driver registration successful\n");

	// Check probing was done
	if (ret != 0) {
		printk(KERN_ERR "%s:%s() error registering driver\n", __FILE__, __FUNCTION__);
		goto error_cleanup;
	}

	return 0;

error_cleanup:
	kfree(pdx);
	platform_device_unregister(pdx->dev);
	return -ENOENT;
}

static void __exit warpfpga_cleanup_module(void)
{
#ifdef WARP_V2
	platform_driver_unregister(&warp_fpga_platform_driver);
#else
	pci_unregister_driver(&warp_fpga_pci_driver);
#endif
	platform_device_unregister(pdx->dev);

	kfree(pdx);

	printk(KERN_INFO "WARP FPGA driver unregistered\n");
}

module_init(warpfpga_init_module);
module_exit(warpfpga_cleanup_module);

MODULE_AUTHOR("Pawel Pastuszak, Alexandre Fournier");
MODULE_DESCRIPTION("WARP V2/V3 FPGA Driver Core " PikaLoadVerString);
MODULE_LICENSE("GPL");
