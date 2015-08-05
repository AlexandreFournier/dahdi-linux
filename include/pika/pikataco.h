#ifndef __PIKATACO_H__
#define __PIKATACO_H__
#ifdef CONFIG_WARP

#include <linux/list.h>

/* DMA defines */
#define FRAMESPERBUFFER		320 /* 40ms / (125us per frame) */
#define FRAMESIZE		64

/* FPGA defines */
#define FPGA_CONFIG                   0x0000 /* Config */
#define FPGA_UPDATE                   0x0004 /* Update */
#define FPGA_PFT                      0x000C /* Termincation on BRI */
#define FPGA_RESET                    0x0014 /* Reset control */
#define FPGA_DIAG                     0x0018 /* Diagnostics */
#define FPGA_REV                      0x001C /* FPGA load type and revision */
#define FPGA_CIAR                     0x0024 /* Codec Indirect Register */
#define FPGA_WRITE_PROTECT            0x0028
#define FPGA_ICCR                     0x002c /* Interrupt Cause/Clear */
#define FPGA_IMR                      0x0030 /* Interrupt Mask */
#define FPGA_STAT_LINES               0x0034 /* Status Lines */

#define FPGA_TDM_DMA_CTRL             0x0E00 /* TDM DMA Control/Status */
#define FPGA_TDM_DMA_HOST_PTR         0x0E04 /* TDM DMA Current Host Buffer */
#define FPGA_TDM_DMA_SZ               0x0E08 /* TDM DMA Transfer Size (frames) */
#define FPGA_TDM_DMA_JIT              0x0E0C /* TDM DMA Jitter Buffer (frames) */
#define FPGA_DMA_SG_IDX               0x0E10 /* DMA SG List Index Register */
#define DEC_DMA_CTRL                  0x0F00 /* DEC DMA Control/Status */
#define DEC_DMA_HOST_PTR              0x0F04 /* DEC DMA Current Host Buffer */
#define DEC_DMA_SZ                    0x0F08 /* DEC DMA Transfer Size (frames) */
#define DEC_DMA_JIT                   0x0F0C /* DEC DMA Jitter Buffer (frames) */
#define DEC_DMS_SG_IDX                0x0F10 /* DEC DMA Scatter Gather List Index */
#define DIAL_STR_ARRAY                0x1000 /* 24 Dial string arrays */
#define START_DIAL                    0x1C00 /* Pulse Dialing Start */
#define STOP_DIAL                     0x1C04 /* Pulse Dialing Stop */
#define PD_MAKE_TIME                  0x1C08 /* Pulse Dial Make Time (ms) */
#define PD_BREAK_TIME                 0x1C0C /* Pulse Dial Break Time (ms) */
#define PD_INTER_DIG_TIME             0x1C10 /* Pulse Dial Inter-Digit Time (ms) */
#define PD_PAUSE_TIME                 0x1C14 /* Pulse Dial Pause Time (ms) */
#define FPGA_SG_LIST_ENTRY            0x2000 /* Scatter Gather List Entry */

/* FPGA defines (masks) */
#define FPGA_PCM_LOOPBACK_BIT         0x00000002
#define FPGA_ICCR_CODEC_READY_MASK    0x00001000
#define FPGA_ICCR_DMA_DONE_MASK       0x00080000


int pikadma_register(int cardid, void *dev, unsigned int irql);
int pikadma_unregister(int cardid);
void pikadma_get_buffers(u8 **rx_buf, u8 **tx_buf);
void pikadma_enable(void);
void pikadma_disable(void);

int fpga_read_indirect(void __iomem *fpga, int channel, int reg, __u8 *value);
int fpga_write_indirect(void __iomem *fpga, int channel, int reg, __u8 value);
void fpga_lock(unsigned long *flags);
void fpga_unlock(unsigned long *flags);

int warp_read_serial(unsigned *serial);

#endif /* CONFIG_WARP */

#define FPGA_RESET_CODEC_BIT	0x04
#define FPGA_RESET_BRI_BIT	0x10
#define FPGA_RESET_SPI_BIT	0x20
#define FPGA_RESET_GSM_BIT	0x40
#define FPGA_RESET_PRI_BIT	0x80

#endif
