#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sections.h>

#include <pika/driver.h>
#include <pika/warp-fpga.h>

#define FPGA_VERSION 		"1.0.1"
#define FPGA_UDEV_NAME		"fpga"

#define PKH_MEDIA_STREAM_BUFFER_SIZE_DEFAULT	(40 * 8) // 40 ms
#define FRAMES_PER_BUFFER			PKH_MEDIA_STREAM_BUFFER_SIZE_DEFAULT
#define FRAMES_PER_TRANSFER			8
#define FRAMESIZE				64

#ifndef VM_RESERVED
#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

static struct warp_fpga * ctx;
static u8 * rx_buf = NULL;
static u8 * tx_buf = NULL;

/**************************************************************/
/* Protypes                                                   */
/**************************************************************/

int create_scatter_gather(struct warp_fpga *dma, int entires, int testmode);
void destroy_scatter_gather(struct warp_fpga *dma, int entries);
void warp_fpga_reconfig(void);

/***************************************************************/
/* File operations                                             */
/***************************************************************/
static int fpga_nopage_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned int size = (FRAMES_PER_BUFFER * FRAMESIZE) * 2;

#if !defined( _M_MPPC  ) && !defined( _M_PPC ) && !defined(PPC) && !defined(__PPC) && !defined(_ARCH_PPC)
	unsigned long pfn;
#endif
	if (vma->vm_pgoff == 0 )
	{
			// Hand out the shared data area 
			printk(KERN_NOTICE "Pika VMA open %p - size: %d \n", ctx->dma_buf, size);
			vma->vm_flags |= VM_IO | VM_RESERVED;

#if defined( _M_MPPC  ) || defined( _M_PPC ) || defined(PPC) || defined(__PPC) || defined(_ARCH_PPC)
			return dma_mmap_coherent(NULL, vma, ctx->rx_buf, ctx->dma_handle, size);
#else
			pfn = page_to_pfn(virt_to_page(ctx->rx_buf));			
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			printk("PFN %08x\n", pfn);
			// pfn + vma->vm_pgoff,
			// vma->vm_end - vma->vm_start,
			return remap_pfn_range(vma, vma->vm_start, ctx->dma_handle >> PAGE_SHIFT, size, vma->vm_page_prot);
#endif
	}
 
	return 0;
}

static int fpga_reg_control(unsigned int cmd_in, unsigned long arg)
{
	struct fpga_ioctl_t fpga;
	
	if (copy_from_user(&fpga, (void*)arg, sizeof(struct fpga_ioctl_t))) {
		printk(KERN_ERR "Error copy data from user fpga_ioctl_t\n");
		return -EFAULT;
	}

	switch (cmd_in) 
	{
		case FPGA_IOCTL_GET_REG:
			fpga.val = fpga_read(ctx->fpga, fpga.reg);
			break;
		case FPGA_IOCTL_SET_REG:
			fpga_write(ctx->fpga, fpga.reg, fpga.val);
			break;
	}

	if (copy_to_user((void *)arg, &fpga, sizeof(struct fpga_ioctl_t))) {
		printk(KERN_ERR " Could not copy fpga_ioctl_t data back to userspace\n");
		return -EFAULT;
	}

	return 0;
}


static int fpga_tx_data(unsigned int cmd_in, unsigned long arg)
{
	struct fpga_ioctl_data_t tx;

	if (copy_from_user(&tx, (void*)arg, sizeof(struct fpga_ioctl_data_t))) {
		printk(KERN_ERR "Error copy data from user fpga_ioctl_data_t\n");
		return -EFAULT;
	}

	//printk(" TX:%p %02x \n",ctx->tx_buf + tx->offset, tx->data);
	if (tx_buf)
		*(tx_buf + tx.offset) = tx.data;
	//pikadma_write_tx(tx->offset, tx->data);

	return 0;
}
static int fpga_rx_data( unsigned int cmd_in, unsigned long arg )
{
	struct fpga_ioctl_data_t rx;

	if (copy_from_user(&rx, (void*)arg, sizeof(struct fpga_ioctl_data_t))) {
		printk(KERN_ERR "Error copy data from user fpga_ioctl_data_t\n");
		return -EFAULT;
	}
	
	if (rx_buf)
		rx.data = *(rx_buf + rx.offset);
	//rx->data = pikadma_read_rx(rx->offset);
	//printk(" RX: %02x \n", tx->data);
	
	if (copy_to_user((void *)arg, &rx, sizeof(struct fpga_ioctl_data_t))) {
		printk(KERN_ERR " Could not copy fpga_ioctl_data_t data back to userspace\n");
		return  -EFAULT;
	}
	
	return 0;
}

static long fpga_ioctl(struct file *file, unsigned int cmd_in, unsigned long arg) 
{
	long rc = 0;

	switch (cmd_in) 
	{
		case FPGA_IOCTL_SET_REG:
		case FPGA_IOCTL_GET_REG:
			rc = fpga_reg_control(cmd_in, arg);
			break;
		case FPGA_IOCTL_WRITE_DATA:
			fpga_tx_data(cmd_in, arg);
			break;
		case FPGA_IOCTL_READ_DATA:
			fpga_rx_data(cmd_in, arg);
		case FPGA_IOCTL_DISABLE_IRQ:
			disable_irq(ctx->irq);
			break;
		case FPGA_IOCTL_ENABLE_IRQ:
			enable_irq(ctx->irq);
			break;
		case FPGA_IOCTL_INIT_TEST_SGL:
			// ???
			create_scatter_gather(ctx, 4, 1);
			destroy_scatter_gather(ctx, 4);
			break;
		case FPGA_IOCTL_REINIT_FPGA:
		{
			warp_fpga_reconfig();
			break;
		}
	}

	return rc;
}

/***************************************************************/
/* UDEV: Setup structures for default setting for udev device  */
/***************************************************************/

static const struct file_operations fpga_fops = {
	.owner		= THIS_MODULE,
	.read		= NULL,
	.mmap 		= fpga_nopage_mmap,
	.unlocked_ioctl	= fpga_ioctl,
};

static struct miscdevice fpga_dev = {
	/*
	 * We don't care what minor number we end up with, so tell the
	 * kernel to just pick one.
	 */
	MISC_DYNAMIC_MINOR,
	/*
	 * Name ourselves /dev/fpga.
	 */
	FPGA_UDEV_NAME,
	/*
	 * What functions to call when a program performs file
	 * operations on the device.
	 */
	&fpga_fops
};

/***************************************************************/
/* INIT                                                        */
/***************************************************************/

int warpfpgactrl_init(void)
{
	int ret = 0;

	ctx = warp_fpga_getdevice();

	/*
	 * Create the "fpga" device in the /sys/class/misc directory.
	 * Udev will automatically create the /dev/fgpa device using
	 * the default rules.
	 */
	ret = misc_register(&fpga_dev);
	if (ret) 
		printk(KERN_ERR "Unable to register \"WARP FPGA\" device driver\n");
	else
		printk("WARP FPGA Device Loaded\n");
	
	pikadma_get_buffers(&rx_buf, &tx_buf);
	printk(" init buffer rx_buf: %p, tx_buf: %p\n", rx_buf, tx_buf);
	return ret;
}
EXPORT_SYMBOL(warpfpgactrl_init);

int warpfpgactrl_remove(void)
{
	misc_deregister(&fpga_dev);
	printk("WARP FPGA Device Removed/Unloaded\n");
	return 0;
}
EXPORT_SYMBOL(warpfpgactrl_remove);
