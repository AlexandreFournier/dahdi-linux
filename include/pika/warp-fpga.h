#ifndef __WARP_FPGA_H__
#define __WARP_FPGA_H__

#include <linux/mutex.h>
#include <linux/miscdevice.h>

#ifdef WARP_V2
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#else
#include <linux/pci.h>
#endif

#include <asm/uaccess.h>
#include <asm/io.h>

/**
 * WARP FPGA private data structure.
 *
 * @base: PCI base address of Memory mapped I/O register.
 * @fpga: ???
 * @sgl:  Scatter-Gather base address
 * @dev:  Pointer to device structure.
 * @lock: Mutex
 * @list: List of callback
 */
struct warp_fpga {
	struct mutex lock;

	struct device *dev;
#ifdef WARP_V2
	struct platform_device * pdev;
#else
	struct pci_dev * pdev;
#endif

	struct list_head list;
	// struct list_head mod_list; // List of callback function to call when mod a or mod triggers

	void __iomem *base;
	void __iomem *fpga;
	void __iomem *sgl;

	u8 *dma_buf;
	u8 *rx_buf, *tx_buf;
	dma_addr_t dma_handle;
	int dma_size;

	int irq;
};

/** IOCTL ***/
struct fpga_ioctl_t 
{
	uint32_t reg;
	uint32_t val;
};

struct fpga_ioctl_data_t
{
	uint32_t offset;
	char data;
};

#define FPGA_IOC_MAGIC                   'F'

#define FPGA_IOCTL_GET_REG               _IO(FPGA_IOC_MAGIC,1)
#define	FPGA_IOCTL_SET_REG               _IO(FPGA_IOC_MAGIC,2)
#define	FPGA_IOCTL_WRITE_DATA            _IO(FPGA_IOC_MAGIC,3)
#define	FPGA_IOCTL_READ_DATA             _IO(FPGA_IOC_MAGIC,4)
#define	FPGA_IOCTL_DISABLE_IRQ           _IO(FPGA_IOC_MAGIC,5)
#define	FPGA_IOCTL_ENABLE_IRQ            _IO(FPGA_IOC_MAGIC,6)
#define	FPGA_IOCTL_INIT_TEST_SGL         _IO(FPGA_IOC_MAGIC,7)
#define	FPGA_IOCTL_REINIT_FPGA           _IO(FPGA_IOC_MAGIC,8)

int pikadma_register_cb(int cardid, void (*cb)(int cardid, void *arg), void * arg);
int pikadma_unregister(int cardid);

unsigned int fpga_read(void __iomem *fpga, int reg);
void fpga_write(void __iomem *fpga, int reg, unsigned value);
u16 fpga_readw(void __iomem *fpga, int reg);
void fpga_writew(void __iomem *fpga, int reg, u16 value);

struct warp_fpga * warp_fpga_getdevice(void);
void warp_fpga_enable_irq(int irq );
void warp_fpga_disable_irq(int irq );
int warp_fpga_isactive(void);
int warp_fpga_set_dsp_callback( void (*f)(int dspid,void *arg) , void * arg  );
int warp_fpga_dsp_count(void);

#endif
