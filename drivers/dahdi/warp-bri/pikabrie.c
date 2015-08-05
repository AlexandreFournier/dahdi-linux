#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>

#include <pika/warp-fpga.h>
#include <pika/driver.h>
#include <pika/daytona.h>
#include <pika/pikataco.h>

#include <dahdi/kernel.h>

#include "xhfc24succ.h"

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
#define USE_TASKLET 1
#endif

#ifndef USE_TASKLET
#include <linux/workqueue.h>
#endif

#ifdef CONFIG_POST
#include <linux/post.h>
#endif

#ifdef WARP_V2
#include <linux/of.h>
#endif

/* FPGA defines */
#define FPGA_CONFIG		0x0000 /* Configuration */
#define FPGA_PFT		0x000C /* Termination on BRI */
#define FPGA_RESET		0x0014 /* Reset control */
#define FPGA_DIAG		0x0018 /* Board Diagnostics */
#define FPGA_REV		0x001C /* FPGA load type and revision */
#define FPGA_CIAR		0x0024 /* Codec Indirect Register */
#define FPGA_ICCR		0x002c /* Interrupt Cause/Clear */
#define FPGA_IMR		0x0030 /* Interrupt Mask */
#define FPGA_STAT_LINES		0x0034 /* Status Lines */
#define FPGA_DMA_SG_IDX		0x0E10 /* DMA SG List Index */

/* XHFC board defines */
#define MAX_XHFC_SPANS          4      /* This is per chip */
#define CHAN_PER_SPAN           4      /* D, B1, B2, PCM */
#define MAX_CHAN (MAX_XHFC_SPANS * CHAN_PER_SPAN)
#define MAX_SPANS		8

/* Flags in _u16  span mode */
#define SPAN_UNUSED		0x0000
#define SPAN_MODE_NT		0x0001
#define SPAN_MODE_TE		0x0002
#define SPAN_MODE_S0		0x0004
#define SPAN_MODE_UP		0x0008
#define SPAN_MODE_EXCH_POL	0x0010
#define SPAN_MODE_LOOP_B1	0x0020
#define SPAN_MODE_LOOP_B2	0x0040
#define SPAN_MODE_LOOP_D	0x0080
#define SPAN_MODE_ENDPOINT	0x0100
#define SPAN_MODE_STARTED       0x1000

#define SPAN_MODE_LOOPS		0xE0	/* mask span mode Loop B1/B2/D */

/* NT / TE defines */
#define CLK_DLY_TE	0x0e	/* CLKDEL in TE mode */
#define CLK_DLY_NT	0x6c	/* CLKDEL in NT mode */
#define STA_ACTIVATE	0x60	/* start activation   in A_SU_WR_STA */
#define STA_DEACTIVATE	0x40	/* start deactivation in A_SU_WR_STA */
#define XHFC_TIMER_T3	2000	/* 2s activation timer T3 */
#define XHFC_TIMER_T4	500	/* 500ms deactivation timer T4 */

/* XHFC Layer1 flags (stored in xhfc_port_t->l1_flags) */
#define HFC_L1_ACTIVATING	1
#define HFC_L1_ACTIVATED	2

#define BRIE_CHANS_PER_SPAN 3

/* DMA - this must match wa_dma.h but we cannot include wa_dma.h */
#define FRAMES_PER_BUFFER	320 /* 40ms / (125us per frame) */
#define FRAMES_PER_TRANSFER	8
#define FRAMESIZE		64
#define DMA_CARD_ID		7

//#define DAHDI_2_4_0		1
#define DAHDI_2_5_0		1
#define DAHDI_2_6_0		1
#define DAHDI_2_8_0		1

#ifdef DAHDI_2_5_0
static int brie_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc);
static int brie_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype);
static int brie_startup(struct file *file,struct dahdi_span *span);
#else
static int brie_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc);
static int brie_chanconfig(struct dahdi_chan *chan, int sigtype);
static int brie_startup(struct dahdi_span *span);
#endif
static int brie_shutdown(struct dahdi_span *span);
static int brie_open(struct dahdi_chan *chan);
static int brie_close(struct dahdi_chan *chan);
static int brie_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data);
static void brie_hdlc_hard_xmit(struct dahdi_chan *chan);

/* warp analog span ops struct for DAHDI 2.4.0 and beyond */
#if defined(DAHDI_2_4_0) || defined (DAHDI_2_5_0)
static const struct dahdi_span_ops warp_bri_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = brie_spanconfig,
	.chanconfig = brie_chanconfig,
	.startup = brie_startup,
	.shutdown = brie_shutdown,
	.open = brie_open,
	.close = brie_close,
	.ioctl = brie_ioctl,
	.hdlc_hard_xmit = brie_hdlc_hard_xmit,
};
#endif

/* span struct for each S/U span */
struct xhfc_span {
	int span_id; /* Physical span id */
	int id; /* 0 based, no gaps */
	int modidx;
	struct xhfc *xhfc;
	struct xhfc_chan *d_chan;
	struct xhfc_chan *b1_chan;
	struct xhfc_chan *b2_chan;

	int timeslot;

	atomic_t open_count;

	/* hdlc transmit data */
	int tx_idx;
	int tx_size;
	int tx_frame;
	u8  tx_buf[128];

	/* hdlc receive data */
	int rx_idx;
	u8  rx_buf[128];

	u16 mode;		/* NT/TE + ST/U */
	u8 state;
	u_long	l1_flags;
	struct timer_list t3_timer;	/* for activation/deactivation */
	struct timer_list t4_timer;	/* for activation/deactivation */
	struct timer_list t1_timer;

	/* Alarm state for dahdi */
	int newalarm;
	struct timer_list alarm_timer;

	/* chip registers */
	reg_a_su_ctrl0 su_ctrl0;
	reg_a_su_ctrl1 su_ctrl1;
	reg_a_su_ctrl2 su_ctrl2;

	/* dahdi */
	struct dahdi_span span;
	struct dahdi_chan *chans[BRIE_CHANS_PER_SPAN]; /* Individual channels */
	struct dahdi_chan *sigchan; /* the signaling channel for this span */

};

/* channel struct for each fifo */
struct xhfc_chan {
	int id;
	struct xhfc_span *span;

	unsigned char writechunk[DAHDI_CHUNKSIZE];
	unsigned char readchunk[DAHDI_CHUNKSIZE];

	struct dahdi_chan chan;
};

struct xhfc {
	__u8 chipnum;           /* global chip number */
	__u8 modidx;            /* module index 0 = mod a, 1 = mod b */
	struct brie *dev;       /* back pointer to g_brie */

	int num_spans;		/* number of S and U interfaces */
	int max_fifo;		/* always 4 fifos per span */
	__u8 max_z;		/* fifo depth -1 */

	struct xhfc_chan *chan;	/* one each D/B/PCM channel */

	/* chip registers */
	reg_r_irq_ctrl		irq_ctrl;
	reg_r_misc_irqmsk	misc_irqmask;	/* mask of enabled interrupt sources */
	reg_r_misc_irq		misc_irq;	/* collect interrupt status bits */

	reg_r_su_irqmsk		su_irqmask;	/* mask of line interface state change interrupts */
	reg_r_su_irq		su_irq;		/* collect interrupt status bits */

	__u32 fifo_irq;		/* fifo bl irq */
	__u32 fifo_irqmask;	/* fifo bl irq */

	/* Debugging */
	u8 r_af0_oview;
	u8 r_bert_sta;
	u32 rx_fifo_errs;
	u32 tx_fifo_errs;

#ifdef DAHDI_2_6_0
	struct dahdi_device *ddev;
#endif

	spinlock_t fifo_tx_lock;		/* held when accessing anything to affection the fifo */
	spinlock_t fifo_lock;                   /* held when accessing anything to affection the fifo */
	//spinlock_t tasklet_lock;

};

struct brie {
	void __iomem *fpga;
	int irq;

	int loopback;

	int pcm_master;
#ifdef USE_TASKLET
	struct tasklet_struct brie_bh;
#else
	struct work_struct brie_work;
#endif
	u8 moda;
	u8 modb;
	u8 num_xhfcs;
	struct xhfc *xhfc;
	struct xhfc_span *spans[MAX_SPANS];

	/* DMA */
	u8 *tx_buf;
	u8 *rx_buf;

	// WARP V3
	struct warp_fpga * dma;
	spinlock_t tasklet_lock;
	spinlock_t workqueue_lock;
};

static struct workqueue_struct *brie_wq;

static int debug = 0 ;
module_param(debug, int, 0664);

static inline __u8 read_xhfc(struct xhfc *xhfc, __u8 reg)
{
	__u8 data = 0;
	//unsigned long flags;
	//spin_lock_irqsave(&xhfc->dev->tasklet_lock, flags);

	fpga_read_indirect(xhfc->dev->fpga, xhfc->chipnum, reg, &data);
	//spin_unlock_irqrestore(&xhfc->dev->tasklet_lock, flags);
	return data;
}

static inline void write_xhfc(struct xhfc *xhfc, __u8 reg, __u8 value)
{
	//unsigned long flags;
	//spin_lock_irqsave(&xhfc->dev->tasklet_lock, flags);
	fpga_write_indirect(xhfc->dev->fpga, xhfc->chipnum, reg, value);
	//spin_unlock_irqrestore(&xhfc->dev->tasklet_lock, flags);
}

static inline void xhfc_waitbusy(struct xhfc *xhfc)
{
	unsigned long start;
	int timeout  = HZ; /* 1s */

	start = jiffies;
	while (read_xhfc(xhfc, R_STATUS) & M_BUSY)
	{
		if (time_after(jiffies,start+timeout))
		{
			printk(KERN_ERR "xhfc_waitbusy: timeout waiting for R_STATUS to not be busy,  R_STATUS: 0x%02x\n", read_xhfc(xhfc, R_STATUS));
			return;
		}
		cpu_relax();
	}

}

static inline void xhfc_selfifo(struct xhfc *xhfc, __u8 fifo)
{
	write_xhfc(xhfc, R_FIFO, fifo);
	xhfc_waitbusy(xhfc);
}

static inline void xhfc_inc_f(struct xhfc *xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_INC_F);
	xhfc_waitbusy(xhfc);
}

static inline __init void xhfc_resetfifo(struct xhfc *xhfc)
{
	write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO | M_RES_FIFO_ERR);
	xhfc_waitbusy(xhfc);
}

static void xhfc_write_fifo(struct xhfc *xhfc, int ch_index);
static void xhfc_read_fifo(struct xhfc *xhfc, int ch_index);

/* layer 1 specific */
static void l1_activate(struct xhfc_span *span);
static void l1_deactivate(struct xhfc_span *span);
static void l1_timer_start_t1(struct xhfc_span *span);
static void l1_timer_start_t3(struct xhfc_span *span);
static void l1_timer_start_t4(struct xhfc_span *span);
static inline void l1_timer_stop_t1(struct xhfc_span *span);
static inline void l1_timer_stop_t3(struct xhfc_span *span);
static inline void l1_timer_stop_t4(struct xhfc_span *span);
static void l1_timer_expire_t1(unsigned long arg);
static void l1_timer_expire_t3(unsigned long arg);
static void l1_timer_expire_t4(unsigned long arg);

static void alarm_timer_update(struct xhfc_span *span);
static void alarm_timer_expire(unsigned long arg);

#ifdef USE_TASKLET
static void brie_tasklet(unsigned long arg);
#else
static void brie_worker(struct work_struct *work);
#endif

/* channel specific */
static void brispan_apply_config(struct xhfc_span *span);
static int brichannels_create(struct xhfc_span *span);
static int dchannel_setup_fifo(struct xhfc_chan *chan, unsigned rx);
static void dchannel_toggle_fifo(struct xhfc_chan *chan, u8 enable);
static int bchannel_setup_pcm(struct xhfc_chan *chan, unsigned rx);
static void brispan_new_state(struct xhfc_span *span,
			      u8 new_state, int expired);
static int bchannel_toggle(struct xhfc_chan *chan, u8 enable);
static int brispan_start(struct xhfc_span *span);
static int brispan_stop(struct xhfc_span *span);

static void brie_disable_interrupts(struct brie *dev);
static unsigned nt_mask;
module_param(nt_mask, uint, 0444);

static unsigned endpoint_mask = 0xff;
module_param(endpoint_mask, uint, 0444);

static int teignored = 1;
module_param(teignored, int, 0664);

/* Only one instance of the driver */
static struct brie *g_brie;

/* -------------------------------------------------------------------------- */
/* ISR and bottom half */

/* ModA = 20, ModB = 21 */
static unsigned bri_int_mask;
/* Underrun = 18, Done = 19 */
#define DMA_INT_MASK ((1 << 18) | (1 << 19))

static irqreturn_t xhfc_interrupt(int irq, void *arg)
{
	struct brie *dev = arg;
	struct xhfc *xhfc = NULL;
	u8 i, j, reg;
	u32 xhfc_irqs;
	//unsigned long flags;

	//spin_lock_irqsave(&dev->tasklet_lock,flags);
	xhfc_irqs = 0;
	for (i = 0; i < dev->num_xhfcs; i++) {
		xhfc = &dev->xhfc[i];
		if (xhfc->irq_ctrl.bit.v_glob_irq_en &&
		    read_xhfc(xhfc, R_IRQ_OVIEW))
			/* mark this xhfc possibly had irq */
			xhfc_irqs |= (1 << i);
	}
	if (!xhfc_irqs)
		return IRQ_NONE;

	xhfc_irqs = 0;

	for (i = 0; i < dev->num_xhfcs; i++) {
		xhfc = &dev->xhfc[i];

		reg = read_xhfc(xhfc, R_MISC_IRQ);
		xhfc->misc_irq.reg |= reg;
		if (reg & 1) {
			xhfc->r_af0_oview = read_xhfc(xhfc, R_AF0_OVIEW);
			xhfc->r_bert_sta = read_xhfc(xhfc, R_BERT_STA);
		}

		xhfc->su_irq.reg |= read_xhfc(xhfc, R_SU_IRQ);

		/* get fifo IRQ states in bundle */
		for (j = 0; j < 4; j++) {
			xhfc->fifo_irq |=
			    (read_xhfc(xhfc, R_FIFO_BL0_IRQ + j) << (j * 8));
		}

		/* call bottom half at events
		 *   - Timer Interrupt (or other misc_irq sources)
		 *   - SU State change
		 *   - Fifo FrameEnd interrupts (only at rx fifos enabled)
		 */
		if ((xhfc->misc_irq.reg & xhfc->misc_irqmask.reg)
		      || (xhfc->su_irq.reg & xhfc->su_irqmask.reg)
		      || (xhfc->fifo_irq & xhfc->fifo_irqmask)) {
			/* mark this xhfc really had irq */
			xhfc_irqs |= (1 << i);

			/* queue bottom half */
#ifdef USE_TASKLET
			tasklet_schedule(&dev->brie_bh);
#else
			queue_work(brie_wq, &dev->brie_work);
#endif
			//brie_tasklet( (unsigned long)dev);
		}
	}
	//spin_unlock_irqrestore(&dev->tasklet_lock, flags);

	return xhfc_irqs ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t brie_isr(int irq, void *arg)
{
	struct brie *dev = arg;
	irqreturn_t handled = IRQ_NONE;
	unsigned iccr = fpga_read(dev->fpga, FPGA_ICCR);

	if (iccr & bri_int_mask)
		handled = xhfc_interrupt(irq, dev);

	return handled;
}

static inline void handle_su_interrupt(struct xhfc *xhfc)
{
	struct xhfc_chan *dch;
	u8 state;
	int i, span_offset;

	xhfc->su_irq.reg = 0;

	span_offset = xhfc->modidx*4;
	for (i = span_offset; i < (span_offset + xhfc->num_spans); i++) {
		struct xhfc_span *span = xhfc->dev->spans[i];
		if (!span)
			continue;

		dch = span->d_chan;

		write_xhfc(xhfc, R_SU_SEL, span->id);
		state = read_xhfc(xhfc, A_SU_RD_STA);

		if (state != span->state)
			brispan_new_state(span, state, 0);
	}
}

static inline void handle_fifo_interrupt(struct xhfc *xhfc, unsigned fifo_irq)
{
	int i;
	/* Handle rx fifos */
	for (i = 0; i < xhfc->max_fifo; i++) {
		if (fifo_irq & (1 << (i * 2 + 1))) {

			xhfc->fifo_irq &= ~(1 << (i * 2 + 1));
			xhfc_read_fifo(xhfc, i);
		}
	}

	for (i = 0; i < xhfc->max_fifo; i++)
	{
		if (fifo_irq & (1 << (i * 2))) {
			xhfc->fifo_irq &= ~(1 << (i * 2));
			xhfc_write_fifo(xhfc, i);
		}
	} 
}

#ifdef USE_TASKLET
static void brie_tasklet(unsigned long arg)
#else
static void brie_worker(struct work_struct *work)
#endif
{
#ifdef USE_TASKLET
	struct brie *dev = (struct brie *)arg;
#else	
	struct brie *dev = container_of(work, struct brie, brie_work);
#endif
	struct xhfc *xhfc;
	unsigned fifo_irq;
	int i, x;

	//unsigned long flags;
	//spin_lock_irqsave(&dev->workqueue_lock, flags);

	/* run through once for each xhfc */
	for (x = 0, xhfc = dev->xhfc; x < dev->num_xhfcs; ++x, ++xhfc) {
		/* set fifo_irq when RX data over threshold */
		for (i = 0; i < xhfc->num_spans; i++)
			xhfc->fifo_irq |=
				read_xhfc(xhfc, R_FILL_BL0 + i) << (i * 8);
		
		fifo_irq = xhfc->fifo_irq & xhfc->fifo_irqmask;
		if (fifo_irq)
			handle_fifo_interrupt(xhfc, fifo_irq);
		
		if (xhfc->su_irq.reg & xhfc->su_irqmask.reg)
			handle_su_interrupt(xhfc); 
	}

	//spin_unlock_irqrestore(&dev->workqueue_lock, flags);
}

/* -------------------------------------------------------------------------- */
/* DAHDI interface functions */

/* Called by dahdi_cfg program. */
#ifdef DAHDI_2_5_0
static int brie_startup(struct file *file,struct dahdi_span *span)
#else
static int brie_startup(struct dahdi_span *span)
#endif
{
#if defined(DAHDI_2_4_0) || defined( DAHDI_2_5_0 )
	struct xhfc_span *xspan = container_of(span, struct xhfc_span, span);
	return brispan_start(xspan);
#else
	return brispan_start(span->pvt);
#endif
}

static int brie_shutdown(struct dahdi_span *span)
{	/* Never called */
	return 0;
}

static int brie_open(struct dahdi_chan *chan)
{
	if (!try_module_get(THIS_MODULE))
		return -EBUSY;

	return 0;
}

static int brie_close(struct dahdi_chan *chan)
{
	module_put(THIS_MODULE);
	return 0;
}

static int brie_ioctl(struct dahdi_chan *chan,
		      unsigned int cmd, unsigned long data)
{
	switch (cmd) {
	case DAHDI_TONEDETECT:
		/* We don't do tone detection. */
		return -EINVAL;

	default:
		printk(KERN_INFO "Unexpected ioctl %x\n", cmd); /* SAM DBG */
		return -ENOTTY;
	}

	return 0;
}

/* DAHDI calls this when it has data it wants to send to the HDLC controller */
static void brie_hdlc_hard_xmit(struct dahdi_chan *chan)
{
	struct xhfc_chan *xchan = chan->pvt;
	struct xhfc_span *span = xchan->span;

	if (span->sigchan == chan) {
		/* Kick it */
		span->xhfc->fifo_irq |= 1 << (span->d_chan->id * 2);
		
		// xhfc_write_fifo(span->xhfc,span->d_chan->id  );
#ifdef USE_TASKLET
		tasklet_schedule(&span->xhfc->dev->brie_bh);
#else
		queue_work(brie_wq, &span->xhfc->dev->brie_work);
#endif
		
	} else
		printk(KERN_WARNING "WARNING: %s called on %s\n",
		       __func__, chan->name);
}

/* Called by dahdi_cfg program. */
#ifdef DAHDI_2_5_0
static int brie_spanconfig(struct file *file,struct dahdi_span *span, struct dahdi_lineconfig *lc)
#else
static int brie_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc)
#endif
{
	struct xhfc_span *xspan = container_of(span, struct xhfc_span, span);

	if (lc->lineconfig & DAHDI_CONFIG_NTTE) {
		xspan->mode = SPAN_MODE_NT;

	} else {
		xspan->mode = SPAN_MODE_TE;
	}

	if (lc->lineconfig &  DAHDI_CONFIG_TERM) {
		xspan->mode |= SPAN_MODE_ENDPOINT;
	} else {
		xspan->mode &= ~ SPAN_MODE_ENDPOINT;
	}
	
	printk(KERN_INFO "%s %s span %d lineconfig %x - %s (TERM: %s) \n",
	       __func__, span->name, lc->span, lc->lineconfig, (xspan->mode & SPAN_MODE_NT) ? "NT" : "TE", ( xspan->mode & SPAN_MODE_ENDPOINT ) ? "Enabled" : "Disabled"  );


	// Update settings
#ifdef DAHDI_2_8_0
	span->spantype = (xspan->mode & SPAN_MODE_TE) ? SPANTYPE_DIGITAL_BRI_TE : SPANTYPE_DIGITAL_BRI_NT;
#else
	span->spantype = (xspan->mode & SPAN_MODE_TE) ? "TE" : "NT";
#endif

	sprintf(span->desc, "PIKA BRI_%s Module %c Span %d",
			(xspan->mode & SPAN_MODE_TE) ? "TE" : "NT",
			xspan->xhfc->modidx + 'A', xspan->id);

	// Reset start
	//brispan_stop(xspan);
	//brispan_start(xspan);

	return 0;
}

/* Called by dahdi_cfg program on the d-channels only. */
#ifdef DAHDI_2_5_0
static int brie_chanconfig(struct file *file,struct dahdi_chan *chan, int sigtype)
#else
static int brie_chanconfig(struct dahdi_chan *chan, int sigtype)
#endif
{
	return 0;
}

/* -------------------------------------------------------------------------- */

/* Get a buffer full from dahdi */
static int dahdi_get_buf(struct xhfc_span *span)
{
	int res, size = sizeof(span->tx_buf);
	
	if (span == NULL)
		return 0;

	/* res == 0 if there is not a complete frame. */
	res = dahdi_hdlc_getbuf(span->sigchan, span->tx_buf, &size);

	if (size > 0) {
		span->tx_idx = 0;
		span->tx_size = size;
		span->tx_frame = res;
	}

	return size;
}

/* Takes data from DAHDI and shoots it out to the hardware.  The blob
 * may or may not be a complete HDLC frame.
 */
static void xhfc_write_fifo(struct xhfc * xhfc, int ch_index)
{
	u8 fcnt, tcnt, i;
	u8 free;
	u8 f1, f2;
	u8 fstat;
	u8 *data;
	int remain;
	struct xhfc_chan *ch = xhfc->chan + ch_index;
	struct xhfc_span *span = ch->span;
	//unsigned int flags=0;
	u8 run = 1;

	if (ch == NULL)
		return;

	if (span == NULL)
		return;

	spin_lock(&xhfc->fifo_lock);
	/* if we're ignoring TE red alarms and we are in alarm restart the S/T state machine */
	if (span->mode & SPAN_MODE_TE && teignored && span->newalarm != 0 )
	{
		l1_activate(span);
	}

	if (span->tx_idx == 0)
		if (dahdi_get_buf(span) == 0)
		{

			spin_unlock(&xhfc->fifo_lock);
			return; /* nothing to send */
		}

	while (run)
	{
		remain = span->tx_size - span->tx_idx;

		if ( remain <= 0 ) {
			spin_unlock(&xhfc->fifo_lock);
			return;
		}
	
		xhfc_selfifo(xhfc, ch_index * 2);

		fstat = read_xhfc(xhfc, A_FIFO_STA);
		free = xhfc->max_z - read_xhfc(xhfc, A_USAGE);
		tcnt = free >= remain ? remain : free;

		f1 = read_xhfc(xhfc, A_F1);
		f2 = read_xhfc(xhfc, A_F2);

		fcnt = 0x07 - ((f1 - f2) & 0x07); /* free frame count in tx fifo */
		
		
		/*pr_info("%s channel(%i) len(%i) idx(%i) f1(%i) f2(%i) fcnt(%i)"
				"tcnt(%i) free(%i) fstat(%i)\n",
				__FUNCTION__, ch_index, span->tx_size, span->tx_idx,
				 f1, f2, fcnt, tcnt, free, fstat);
		*/

		
		/* check for fifo underrun during frame transmission */
		fstat = read_xhfc(xhfc, A_FIFO_STA);
		if (fstat & M_FIFO_ERR) {
			pr_err("%s transmit fifo channel(%i) underrun idx(%i),"
				 "A_FIFO_STA(0x%02x)\n",
				 __FUNCTION__, ch_index, span->tx_size, fstat);

			write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
			xhfc->tx_fifo_errs++;

			/* restart frame transmission */
			span->tx_idx = 0;
			continue;
		}


		if (free && fcnt && tcnt) {

			data = span->tx_buf + span->tx_idx;
			span->tx_idx += tcnt;

			/* write data to FIFO */
			//printk(" --- DATA WRITE: %d ch_index: %d\n", tcnt, ch_index);
			for (i = 0; i < tcnt; ++i)
			{
				write_xhfc(xhfc, A_FIFO_DATA, *(data+i));
			//	printk(" %02x ", *data);
			}
			//printk("\n---END---\n");

			if (span->tx_idx == span->tx_size) {
				//pr_info("%s: span->tx_frame: %d\n", __FUNCTION__,span->tx_frame);
				//if (span->tx_frame) {
					/* terminate frame */
					xhfc_inc_f(xhfc);
				//}
		
				span->tx_idx = 0;

				/* check for fifo underrun during frame transmission */
				fstat = read_xhfc(xhfc, A_FIFO_STA);
				if (fstat & M_FIFO_ERR) {
					write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
					xhfc->tx_fifo_errs++;

					/* restart frame transmission */
					span->tx_idx = 0;
					continue;
				}

				/* no underrun, go to next frame if there is one */
				//if (dahdi_get_buf(span) && (free - tcnt) > 8)
				{
					run =0;
					//break;
					//continue;
				}
			} else {
				/* tx buffer not complete, but fifo filled to maximum */
				xhfc_selfifo(xhfc, (ch->id * 2));
				run = 0;
				break;
			}
		} else {
			run = 0;
		}
	}
	spin_unlock(&xhfc->fifo_lock);
}

static void xhfc_read_fifo(struct xhfc *xhfc, int ch_index)
{
	u8	f1, f2, z1, z2;
	u8	fstat = 0;
	int	i;
	int	rcnt;		/* read rcnt bytes out of fifo */
	u8	*data;		/* new data pointer */
	struct xhfc_chan *ch = xhfc->chan + ch_index;
	struct xhfc_span *span = ch->span;
	
	if ( ch == NULL )
		return;

	if ( span == NULL )
		return;

	spin_lock(&xhfc->fifo_lock);
//receive_buffer:
	do {

		xhfc_selfifo(xhfc, ch_index * 2 + 1);

		fstat = read_xhfc(xhfc, A_FIFO_STA);
		if (fstat & M_FIFO_ERR) {
			write_xhfc(xhfc, A_INC_RES_FIFO, M_RES_FIFO_ERR);
			xhfc->rx_fifo_errs++;
		}

		/* hdlc rcnt */
		f1 = read_xhfc(xhfc, A_F1);
		f2 = read_xhfc(xhfc, A_F2);
		z1 = read_xhfc(xhfc, A_Z1);
		z2 = read_xhfc(xhfc, A_Z2);
		
		rcnt = (z1 - z2) & xhfc->max_z;
		if (f1 != f2)
			rcnt++;

		if ((span->rx_idx + rcnt) > sizeof(span->rx_buf))
			rcnt = sizeof(span->rx_buf) - span->rx_idx;


		if (rcnt <= 0) {
			spin_unlock(&xhfc->fifo_lock);
			return; /* nothing to read */
		}

		/* There seems to be a bug in the chip where if we read only
		 * the first byte of a new frame, it gets corrupted. We see
		 * this about once an hour under a fairly systained 16kbps
		 * load. */
		if (span->rx_idx == 0 && rcnt == 1) {
			spin_unlock(&xhfc->fifo_lock);
			return;
		}

		data = span->rx_buf + span->rx_idx;

		/* read data from FIFO */
		//printk("----DATA--%d-\n",rcnt);
		for (i = 0; i < rcnt; ++i) {
				data[i] = read_xhfc(xhfc, A_FIFO_DATA);
		//		printk(" %02x ",data[i]);
		}
		//printk("\n----END DATA---\n");
		span->rx_idx += rcnt;

		if (f1 == f2) {
			xhfc_selfifo(xhfc, ch_index * 2 + 1);
			spin_unlock(&xhfc->fifo_lock);
			return; /* not a frame */
		}
		/* end of frame */
		xhfc_inc_f(xhfc);

		/* check minimum frame size */
		if (span->rx_idx < 4)
			span->rx_idx = 0;
		else {
			/* check crc */
			if (span->rx_buf[span->rx_idx - 1])
			{
				span->rx_idx = 0;
			} else {

				/* remove cksum */
				span->rx_idx -= 1;
		//		printk("-Sending to hdlc\n");
				/* Tell dahdi about the frame. We do not send up bad frames. */
				dahdi_hdlc_putbuf(span->sigchan, span->rx_buf, span->rx_idx);
				dahdi_hdlc_finish(span->sigchan);
			}
		}

//read_exit:
		span->rx_idx = 0;

	} while (read_xhfc(xhfc, A_USAGE) > 8);


	spin_unlock(&xhfc->fifo_lock);

}

/* -------------------------------------------------------------------------- */
/* DMA */

/* prepare incoming audio data to be received by asterisk */
void brie_receiveprep(struct xhfc_span *span, u8 *rx_addr)
{
	int i;
	/* Offset to start of b1 */
	rx_addr += span->timeslot;

	/* read data from DMA buffer */
	for (i = 0; i < DAHDI_CHUNKSIZE; i++) {
		span->b1_chan->readchunk[i] = *rx_addr;
		span->b2_chan->readchunk[i] = *(rx_addr + 1);
		rx_addr += FRAMESIZE;
	}
	/* this is where echo cancellation is done with the registered
	 * echo canceler
	 */
	dahdi_ec_span(&span->span);

	/* push data into asterisk */
	dahdi_receive(&span->span);
}

/* prepare data from asterisk for transmission (put it out on the buffer) */
void brie_transmitprep(struct xhfc_span *span, u8 *tx_addr)
{
	int i;
	
	/* Offset to start of b1 */
	tx_addr += span->timeslot;

	/* compute transmission (get buffer from Asterisk) */
	dahdi_transmit(&span->span);

	/* write data to DMA buffer */
	for (i = 0; i < DAHDI_CHUNKSIZE; i++) {
		*tx_addr       = span->b1_chan->writechunk[i];
		*(tx_addr + 1) = span->b2_chan->writechunk[i];
		tx_addr += FRAMESIZE;
	}
}
#ifdef WARP_V2
static void brie_dma_cb(int cardid)
#else
static void brie_dma_cb(int cardid, void * arg)
#endif
{
	const unsigned int bytes = FRAMES_PER_TRANSFER * FRAMESIZE;
	const unsigned int bcount = FRAMES_PER_BUFFER / FRAMES_PER_TRANSFER;
	u8 *cur_rx_buf, *cur_tx_buf;
	int rx_index, tx_index, index;

	/* get the dma index */
	index = fpga_read(g_brie->fpga, FPGA_DMA_SG_IDX) & 0x1ff;
	index = index >> 1;

	/* pick the RX buffer before the current one */
	rx_index = ((index - 1) >= 0) ? (index - 1) : (index - 1 + bcount);

	/* pick the TX buffer after the current one */
	tx_index = ((index + 1) < bcount) ? (index + 1) : (index + 1 - bcount);

	/* calculate the position of the rx and tx buffers */
	cur_tx_buf = g_brie->tx_buf + tx_index * bytes;
	cur_rx_buf = g_brie->rx_buf + rx_index * bytes;
	
	
	for (index = 0; index < MAX_SPANS; ++index)
		if (g_brie->spans[index]) {
			/* do your receive work */
			brie_receiveprep(g_brie->spans[index], cur_rx_buf);
			/* do your transmit work */
			brie_transmitprep(g_brie->spans[index], cur_tx_buf);
		}
}

static int __init brie_dma_enable(struct brie *dev)
{
#ifdef WARP_V2
	return pikadma_register_cb(DMA_CARD_ID, brie_dma_cb);
#else
	printk("brie_dma_enable entering\n");
	return pikadma_register_cb(DMA_CARD_ID, brie_dma_cb, dev);
#endif
}

/* -------------------------------------------------------------------------- */

/* Register at the same time? */
static void __init dahdi_span_init(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;
	struct dahdi_span *dahdi = &span->span;
#ifdef DAHDI_2_4_0
	struct brie *dev = xhfc->dev;
	dahdi->irq = dev->irq;
#endif
#if defined(DAHDI_2_4_0) || defined(DAHDI_2_5_0)
	/* we use container_of() macro and don't want pvt element in the dahdi struct anymore */
#else
	dahdi->pvt = span;
#endif
	if (debug)
		printk("dahdi_span_init ---\n");

#ifdef DAHDI_2_8_0
	dahdi->spantype = (span->mode & SPAN_MODE_TE) ? SPANTYPE_DIGITAL_BRI_TE : SPANTYPE_DIGITAL_BRI_NT;
#else
	dahdi->spantype = (span->mode & SPAN_MODE_TE) ? "TE" : "NT";
#endif
	dahdi->offset = span->id;
	dahdi->channels = BRIE_CHANS_PER_SPAN;
	dahdi->flags = 0;

	dahdi->deflaw = DAHDI_LAW_ALAW;

	/* For simplicity, we'll accept all line modes since BRI
	 * ignores this setting anyway.*/
	dahdi->linecompat = DAHDI_CONFIG_AMI |
		DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 |
		DAHDI_CONFIG_ESF | DAHDI_CONFIG_HDB3 |
		DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4 | DAHDI_CONFIG_NTTE | DAHDI_CONFIG_TERM;

	sprintf(dahdi->name, "BRI/%d/%d", xhfc->modidx, span->id + 1);
	/* Dahdi matches on this to tell if it is BRI */
	sprintf(dahdi->desc, "PIKA BRI_%s Module %c Span %d",
		(span->mode & SPAN_MODE_TE) ? "TE" : "NT",
		xhfc->modidx + 'A', span->id + 1);
#ifndef DAHDI_2_6_0
	dahdi->manufacturer = "PIKA Technologies Inc.";
	dahdi_copy_string(dahdi->devicetype, "PIKA WARP BRI", sizeof(dahdi->devicetype));
	sprintf(dahdi->location, "Module %c", xhfc->modidx + 'A');
#endif

#if defined(DAHDI_2_4_0) || defined( DAHDI_2_5_0 )
	dahdi->ops = &warp_bri_span_ops;
#else
	dahdi->spanconfig = brie_spanconfig;
	dahdi->chanconfig = brie_chanconfig;
	dahdi->startup = brie_startup;
	dahdi->shutdown = brie_shutdown;
	dahdi->open = brie_open;
	dahdi->close = brie_close;
	dahdi->ioctl = brie_ioctl;
	dahdi->hdlc_hard_xmit = brie_hdlc_hard_xmit;
#endif

	dahdi->chans = span->chans;
#ifdef DAHDI_2_4_0
	init_waitqueue_head(&dahdi->maintq);
#endif
}

static void brie_disable_interrupts(struct brie *dev)
{
	struct xhfc *xhfc;
	int i;

	for (xhfc = dev->xhfc, i = 0; i < dev->num_xhfcs; i++, ++xhfc) {
		/* disable global interrupts */
		xhfc->irq_ctrl.bit.v_glob_irq_en = 0;
		xhfc->irq_ctrl.bit.v_fifo_irq_en = 0;
		write_xhfc(xhfc, R_IRQ_CTRL, xhfc->irq_ctrl.reg);
	}
}

static int brie_stop(struct brie *dev)
{
	int i;

	for (i = 0; i < MAX_SPANS; ++i)
		if (dev->spans[i]) {
			dev->spans[i]->mode = SPAN_MODE_TE; /* reset to TE */
			brispan_stop(dev->spans[i]);
		}

	/* We must disable *after* brispan_stop */
	brie_disable_interrupts(dev);
	pikadma_unregister(DMA_CARD_ID);

	dev->loopback = 0;

	return 0;
}

/* We don't need to cleanup here.. if this fails brispan_cleanup_spans
 * will be called and will cleanup.
 */
static int __init brispan_create(struct xhfc *xhfc, unsigned span_id)
{
	int rc;
	struct xhfc_span *span = kzalloc(sizeof(struct xhfc_span), GFP_KERNEL);

	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	if (!span) {
		printk(KERN_ERR "Unable to create span %d\n", span_id);
		return -ENOMEM;
	}

	/* Calc the span_id for events to user mode. */
	span->span_id = span_id;
	if (xhfc->modidx == 1)
		span->span_id += 4;

	/* Calc the base timeslot */
#ifdef FPGA_6052
	if (xhfc->modidx == 1)
		span->timeslot = ((span->span_id-4) * 2) + 32;
	else
		span->timeslot = span->span_id * 2;
	printk("brispan_create: span_id: %d timeslot: %d\n", span->span_id, span->timeslot);
#else
	span->timeslot = 4 + span->span_id * 2;
#endif
	span->id = span_id;
	span->xhfc = xhfc;
	span->modidx = xhfc->modidx;

	span->mode = (nt_mask & (1 << span->span_id)) ?
		SPAN_MODE_NT : SPAN_MODE_TE;
	if (endpoint_mask & (1 << span->span_id))
		span->mode |= SPAN_MODE_ENDPOINT;
	span->rx_idx = 0;
	span->tx_idx = 0;

	/* init t1 timer - nt only */
	init_timer(&span->t1_timer);
	span->t1_timer.data = (long)span;
	span->t1_timer.function = l1_timer_expire_t1;

	/* init t3 timer - te only */
	init_timer(&span->t3_timer);
	span->t3_timer.data = (long)span;
	span->t3_timer.function = l1_timer_expire_t3;

	/* init t4 timer - te only */
	init_timer(&span->t4_timer);
	span->t4_timer.data = (long) span;
	span->t4_timer.function = l1_timer_expire_t4;

	/* init dahdi alarm timer */
	init_timer(&span->alarm_timer);
	span->alarm_timer.data = (long)span;
	span->alarm_timer.function = alarm_timer_expire;

	xhfc->dev->spans[span->span_id] = span;

	/* spans manage channels */
	rc = brichannels_create(span);
	if (rc)
		return rc;

	dahdi_span_init(span);

	span->sigchan = &span->d_chan->chan;
	if (debug)
		printk("%s: exited\n", __FUNCTION__);


	return 0;
}

static void brispan_destroy(struct xhfc_span *span)
{
	kfree(span);
}

/* Yes this is an init function. It is called to cleanup all the span
 * instances on an xhfc that where created when a span instance fails.
 */
static void __init brispan_cleanup_spans(struct xhfc *xhfc)
{
	int i;

	for (i = 0; i < MAX_SPANS; ++i)
		if (xhfc->dev->spans[i])
			if (xhfc->dev->spans[i]->xhfc == xhfc) {
				brispan_destroy(xhfc->dev->spans[i]);
				xhfc->dev->spans[i] = NULL;
			}
}

static int brispan_start(struct xhfc_span *span)
{
	if (span->mode & SPAN_MODE_STARTED)
		return 0;

	if (debug)
		printk(KERN_INFO "%s %s\n", __func__, span->span.name);

	span->tx_idx = span->rx_idx = 0;

	/* We must set the span in the correct deactivated state for
	 * it's mode */
	write_xhfc(span->xhfc, R_SU_SEL, span->id);
	write_xhfc(span->xhfc, A_SU_WR_STA, (span->mode & SPAN_MODE_TE) ? 0x53 : 0x51);
	udelay(6);

	brispan_apply_config(span);

	dchannel_toggle_fifo(span->d_chan, 1);
	bchannel_toggle(span->b1_chan, 1);
	bchannel_toggle(span->b2_chan, 1);

	l1_activate(span);

	span->span.flags |= DAHDI_FLAG_RUNNING;
	span->mode |= SPAN_MODE_STARTED;

	return 0;
}

static int brispan_stop(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);

	dchannel_toggle_fifo(span->d_chan, 0);

	bchannel_toggle(span->b1_chan, 0);
	bchannel_toggle(span->b2_chan, 0);

	l1_deactivate(span);

	span->mode &= ~SPAN_MODE_STARTED;
	span->span.flags &= ~DAHDI_FLAG_RUNNING;

	/* These actions disable the automatic state machine No state
	   machine changes can occur again until l1_activate is
	   triggered */
	if (span->mode & SPAN_MODE_TE) {
		span->state = 3;
		write_xhfc(xhfc, R_SU_SEL, span->id);
		/* Manually load deactivated state (3 for TE), and set
		 * deactivating flag */
		write_xhfc(xhfc, A_SU_WR_STA, 0x53);
	} else {
		span->state = 3;
		write_xhfc(xhfc, R_SU_SEL, span->id);
		/* Manually load deactivated state (1 for NT), and set
		 * deactivating flag */
		write_xhfc(xhfc, A_SU_WR_STA, 0x51);
	}

	udelay(6);

	/* Make sure all the timers are dead */
	del_timer_sync(&span->t1_timer);
	del_timer_sync(&span->t3_timer);
	del_timer_sync(&span->t4_timer);
	del_timer_sync(&span->alarm_timer);

	return 0;
}

/* Must be called *after* setting TE/NT. */
void set_clock(struct brie *dev)
{
	// unsigned long flags;
	
	struct xhfc *xhfc;
	unsigned syncsrc = 0;
	int i, pcm_master = -1; /* default to none */
	unsigned reg = fpga_read(dev->fpga, FPGA_CONFIG);

	if (debug)
		printk("set_clock entered\n");

	if (!dev->loopback) {
		pcm_master = 0;
		syncsrc = 2 << dev->xhfc[pcm_master].modidx;
	}

	/* Only set the clock if it changed. */
	if (pcm_master == dev->pcm_master && (reg & 6) == syncsrc)
	{
		printk("Already set pcm_master\n");
		return;
	}

	dev->pcm_master = pcm_master;

	/* We cannot allow silabs access while changing the clock.
	 * We must disable the DMA engine while changing the clock.
	 */
	// fpga_lock(&flags);
	pikadma_disable();

	for (i = 0, xhfc = dev->xhfc; i < dev->num_xhfcs; ++i, ++xhfc)
		if (i == pcm_master) {
			printk("Setting PCM_Master on %d\n", i);
			/* short f0i0 pulse, indirect access to 9 of 15 */
			write_xhfc(xhfc, R_PCM_MD0, 0x91);

			/* set pcm to 4mbit/s (64 timeslots) */
			write_xhfc(xhfc, R_PCM_MD1, 0x1C);

			/* indirect register access to A */
			write_xhfc(xhfc, R_PCM_MD0, 0xA1);
			write_xhfc(xhfc, R_PCM_MD2, 0x10);

			/* auto select line for clock source */
			if (dev->num_xhfcs == 1)
				write_xhfc(xhfc, R_SU_SYNC, 0);
			else
				write_xhfc(xhfc, R_SU_SYNC, 0x10);

			/* set pcm to master mode */
			write_xhfc(xhfc, R_PCM_MD0, 0x1);
		} else {
			printk("Setting PCM_Slave on %d\n", i);
			/* short f0i0 pulse, indirect access to 9 of 15 */
			write_xhfc(xhfc, R_PCM_MD0, 0x90);

			/* set pcm to 4mbit/s (64 timeslots) */
			write_xhfc(xhfc, R_PCM_MD1, 0x1C);

			/* indirect register access to A */
			write_xhfc(xhfc, R_PCM_MD0, 0xA0);

			if (!dev->loopback) {
				write_xhfc(xhfc, R_PCM_MD2, 0x00);

				/* use warp as the sync source */
				write_xhfc(xhfc, R_SU_SYNC, 0x00);
			} else {
				write_xhfc(xhfc, R_PCM_MD2, 0x04);

				/* use warp as the sync source */
				write_xhfc(xhfc, R_SU_SYNC, 0x1C);
			}

			/* set pcm to slave mode */
			write_xhfc(xhfc, R_PCM_MD0, 0x00);
		}

	reg = (reg & ~6) | syncsrc;
	fpga_write(dev->fpga, FPGA_CONFIG, reg);

	mdelay(2); /* allow FXO/FXS PLL to settle */

	pikadma_enable();
	// fpga_unlock(&flags);
}

static void fpga_endpoint(struct xhfc_span *span, int unsigned mask)
{
	unsigned reg = fpga_read(span->xhfc->dev->fpga, FPGA_PFT);

	if (span->mode & SPAN_MODE_ENDPOINT)
		reg &= ~mask;
	else
		reg |= mask;

	fpga_write(span->xhfc->dev->fpga, FPGA_PFT, reg);
}

static void fpga_nt_te(struct xhfc_span *span, int unsigned mask)
{
	unsigned reg = fpga_read(span->xhfc->dev->fpga, FPGA_CONFIG);

	if (span->mode & SPAN_MODE_TE)
		reg |= mask;
	else
		reg &= ~mask;

	fpga_write(span->xhfc->dev->fpga, FPGA_CONFIG, reg);
}

static void gpio_endpoint(struct xhfc_span *span, int unsigned mask)
{
	unsigned reg = read_xhfc(span->xhfc, R_GPIO_IN0);

	if (span->mode & SPAN_MODE_ENDPOINT)
		reg &= ~mask;
	else
		reg |= mask;

	write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
}

static void gpio_nt_te(struct xhfc_span *span, int unsigned mask)
{
	unsigned reg = read_xhfc(span->xhfc, R_GPIO_IN0);

	if (span->mode & SPAN_MODE_TE)
		reg |= mask;
	else
		reg &= ~mask;

	write_xhfc(span->xhfc, R_GPIO_OUT0, reg);
}

void brispan_apply_config(struct xhfc_span *span)
{
	unsigned reg_mask;
	u8 delay;
	struct xhfc *xhfc = span->xhfc;

	reg_mask = 8 << (xhfc->modidx + 2 * span->id);


	/* configure te/nt and endpoint */
	switch (span->span_id) {
	case 0:
		fpga_nt_te(span, reg_mask);
		fpga_endpoint(span, 1);
		break;
	case 4:
		fpga_nt_te(span, reg_mask);
		fpga_endpoint(span, 2);
		break;
	case 1:
	case 5:
		fpga_nt_te(span, reg_mask);
		gpio_endpoint(span, 1);
		break;
	case 2:
	case 6:
		gpio_nt_te(span, 2);
		gpio_endpoint(span, 4);
		break;
	case 3:
	case 7:
		gpio_nt_te(span, 8);
		gpio_endpoint(span, 16);
		break;
	}
	set_clock(xhfc->dev);

	/* now xhfc stuff */
	write_xhfc(xhfc, R_SU_SEL, span->id);

	/* Reset state value */
	span->state = 0;

	if (span->mode & SPAN_MODE_NT) {
		delay = CLK_DLY_NT;
		span->su_ctrl0.bit.v_su_md = 1;
		l1_timer_stop_t1(span);
	} else {
		delay = CLK_DLY_TE;
		span->su_ctrl0.bit.v_su_md = 0;
	}

	if (span->mode & SPAN_MODE_EXCH_POL)
		span->su_ctrl2.reg = M_SU_EXCHG;
	else
		span->su_ctrl2.reg = 0;

	/* configure end of pulse control for ST mode (TE & NT) */
	span->su_ctrl0.bit.v_st_pu_ctrl = 1;
	write_xhfc(xhfc, A_ST_CTRL3, 0xf8);

	write_xhfc(xhfc, A_SU_CTRL0, span->su_ctrl0.reg);
	write_xhfc(xhfc, A_SU_CTRL1, span->su_ctrl1.reg);
	write_xhfc(xhfc, A_SU_CTRL2, span->su_ctrl2.reg);

	write_xhfc(xhfc, A_SU_CLK_DLY, delay);

	/* SAM Check if NT? */
	alarm_timer_update(span);
}

void brispan_new_state(struct xhfc_span *span, u8 new_state, int expired)
{
	spin_lock(&span->xhfc->fifo_tx_lock);
	/* Hack for state F6 to F7 change. */
	if ((span->state & 0xf) == 6 && (new_state & 0x4f) == 0x47) {
		u8 state;

		/* We should start a timer, but then
		 * we have lots o' race conditions. */
		mdelay(1);
		write_xhfc(span->xhfc, R_SU_SEL, span->id);
		state = read_xhfc(span->xhfc, A_SU_RD_STA);
		if ((state & 0xf) == 3) {
			/* ignore state 7 */
			new_state = state;
			printk(KERN_INFO
			       "Span %d %s L1 dropping state 7\n",
			       span->span_id,
			       span->mode & SPAN_MODE_TE ? "TE" : "NT");
		}
	}

	if (debug)
		printk(KERN_INFO
		       "Span %d %s L1 from state %2x to %2x (%lx)\n",
		       span->span_id,
		       span->mode & SPAN_MODE_TE ? "TE" : "NT",
		       span->state, new_state,
		       span->l1_flags | (expired << 12));

	span->state = new_state; /* update state now */

	if (span->mode & SPAN_MODE_TE) {

		if ((new_state & 0xf) <= 3 || (new_state & 0xf) >= 7)
			l1_timer_stop_t3(span);

		switch (new_state & 0xf) {
		case 0:			/* TE state F0: Reset */
		case 2:			/* TE state F2: Sensing */
		case 4:			/* TE state F4: Awaiting Signal */
			span->newalarm = DAHDI_ALARM_RED;
			if (debug)
				printk("State: %d - RED ALARM\n", span->newalarm & 0x7);
			break;
		case 5:			/* TE state F5: Identifying Input */
		case 6:			/* TE state F6: Synchronized */
			span->newalarm = DAHDI_ALARM_YELLOW;
			if (debug)
				printk("State: %d - RED YELLOW\n", span->newalarm & 0x7);
			break;

		case 3:			/* TE state F3: Deactivated */
			span->newalarm = DAHDI_ALARM_RED;

			if (debug)
				printk("State: %d - RED ALARM\n", span->newalarm & 0x7);

			if (test_bit(HFC_L1_ACTIVATING, &span->l1_flags))
				l1_activate(span);

			if (test_and_clear_bit(HFC_L1_ACTIVATED, &span->l1_flags))
				l1_timer_start_t4(span);
			break;

		case 7:			/* TE state F7: Activated */
			span->newalarm = 0;

			clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);
			l1_timer_stop_t4(span);

			set_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			if (debug)
				printk("State: %d - ACTIVATED\n", span->newalarm & 0x7);


			break;

		case 8:			/* TE state F8: Lost Framing */
			span->newalarm = DAHDI_ALARM_YELLOW;

			clear_bit(HFC_L1_ACTIVATING, &span->l1_flags);
			l1_timer_stop_t4(span);

			if (debug)
				printk("State: %d - YELLOW ALARM\n", span->newalarm & 0x7);


			break;
		}

		if (debug)
			printk("Sync Source: %d\n", read_xhfc(span->xhfc, R_BERT_STA) & 0x7);

	} else if (span->mode & SPAN_MODE_NT)

		switch (new_state & 0xf) {
		case 0:			/* NT state G0: Reset */
			span->newalarm = DAHDI_ALARM_RED;
			break;

		case 1:			/* NT state G1: Deactivated */
			span->newalarm = DAHDI_ALARM_RED;

			clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			l1_timer_stop_t1(span);
			break;
		case 2:			/* NT state G2: Pending Activation */
			span->newalarm = DAHDI_ALARM_YELLOW;

			if (expired)
				clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);
			else {
				write_xhfc(span->xhfc, R_SU_SEL, span->id);
				write_xhfc(span->xhfc, A_SU_WR_STA,
					   M_SU_SET_G2_G3);

				l1_timer_start_t1(span);
			}
			break;
		case 3:			/* NT state G3: Active */
			span->newalarm = 0;

			set_bit(HFC_L1_ACTIVATED, &span->l1_flags);

			l1_timer_stop_t1(span);
			break;
		case 0x4:		/* NT state G4: Pending Deactivation */
			span->newalarm = DAHDI_ALARM_RED;

			l1_timer_stop_t1(span);
			break;
		}
	
	spin_unlock(&span->xhfc->fifo_tx_lock);
}

static void l1_activate(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	if (test_bit(HFC_L1_ACTIVATED, &span->l1_flags))
		return; /* already activated */

	if (span->mode & SPAN_MODE_TE) {
		set_bit(HFC_L1_ACTIVATING, &span->l1_flags);

		write_xhfc(xhfc, R_SU_SEL, span->id);
		write_xhfc(xhfc, A_SU_WR_STA, STA_ACTIVATE);
		l1_timer_start_t3(span);
	} else {
		write_xhfc(xhfc, R_SU_SEL, span->id);
		write_xhfc(xhfc, A_SU_WR_STA, STA_ACTIVATE | M_SU_SET_G2_G3);
	}
}

/* This function is meant for deactivations that occur during certain
 * state machine timeouts, it is not meant to deactivate the state
 * machine. See brispan_stop to do that */
static void l1_deactivate(struct xhfc_span *span)
{
	struct xhfc *xhfc = span->xhfc;

	if (span->mode & SPAN_MODE_TE) {
		write_xhfc(xhfc, R_SU_SEL, span->id);
		//write_xhfc(xhfc, A_SU_WR_STA, STA_DEACTIVATE);
		write_xhfc(xhfc, A_SU_WR_STA, M_SU_LD_STA | 0x03);
		udelay(6);
		write_xhfc(xhfc, A_SU_WR_STA, 0);

	}
}

static void l1_timer_start_t1(struct xhfc_span *span)
{
	if (!timer_pending(&span->t1_timer)) {
		span->t1_timer.expires =
			jiffies + msecs_to_jiffies(100);
		if (span->mode & SPAN_MODE_STARTED)
			add_timer(&span->t1_timer);
	}
}

static inline void l1_timer_stop_t1(struct xhfc_span *span)
{
	del_timer(&span->t1_timer);
}

static void l1_timer_expire_t1(unsigned long arg)
{
	struct xhfc_span *span = (struct xhfc_span *)arg;
	brispan_new_state(span, span->state, 1);
}

static void l1_timer_start_t3(struct xhfc_span *span)
{
	if (!timer_pending(&span->t3_timer)) {
		span->t3_timer.expires =
			jiffies + msecs_to_jiffies(XHFC_TIMER_T3);
		if (span->mode & SPAN_MODE_STARTED)
			add_timer(&span->t3_timer);
	}
}

static inline void l1_timer_stop_t3(struct xhfc_span *span)
{
	del_timer(&span->t3_timer);
}

static void l1_timer_expire_t3(unsigned long arg)
{
	struct xhfc_span *span = (struct xhfc_span *)arg;
	l1_deactivate(span);
	/* afterwards will attempt to reactivate in state F3 since
	 * HFC_L1_ACTIVATING is set */
}

static void l1_timer_start_t4(struct xhfc_span *span)
{
	if (!timer_pending(&span->t4_timer)) {
		span->t4_timer.expires =
			jiffies + msecs_to_jiffies(XHFC_TIMER_T4);
		if (span->mode & SPAN_MODE_STARTED)
			add_timer(&span->t4_timer);
	}
}

static inline void l1_timer_stop_t4(struct xhfc_span *span)
{
	del_timer(&span->t4_timer);
}

static void l1_timer_expire_t4(unsigned long arg)
{
	struct xhfc_span *span = (struct xhfc_span *)arg;
	clear_bit(HFC_L1_ACTIVATED, &span->l1_flags);
}

static void alarm_timer_update(struct xhfc_span *span)
{
	unsigned long expires = jiffies + msecs_to_jiffies(500);

	if (timer_pending(&span->alarm_timer))
		mod_timer(&span->alarm_timer, expires);
	else {
		span->alarm_timer.expires = expires;
		if (span->mode & SPAN_MODE_STARTED)
			add_timer(&span->alarm_timer);
	}
}

static void alarm_timer_expire(unsigned long arg)
{
	struct xhfc_span *span = (struct xhfc_span *)arg;

	if (span->span.alarms != span->newalarm) {
		span->span.alarms = span->newalarm;
		if ((!span->newalarm && teignored) || (!teignored))
			dahdi_alarm_notify(&span->span);
	}
}

void dchannel_toggle_fifo(struct xhfc_chan *chan, u8 enable)
{
	struct xhfc *xhfc = chan->span->xhfc;
	unsigned fifo = chan->id * 2;
	unsigned mask = (1 << fifo) | (1 << (fifo + 1));

	if (enable)
		xhfc->fifo_irqmask |= mask;
	else
		xhfc->fifo_irqmask &= ~mask;
}

int bchannel_toggle(struct xhfc_chan *chan, u8 enable)
{
	struct xhfc_span *span = chan->span;
	int bc = chan->id % CHAN_PER_SPAN;

	if (bc < 0 || bc > 1)
		return EINVAL;

	/* Force to 1 or 0 */
	enable = enable ? 1 : 0;

	if (bc) {
		span->su_ctrl2.bit.v_b2_rx_en = enable;
		span->su_ctrl0.bit.v_b2_tx_en = enable;
	} else {
		span->su_ctrl2.bit.v_b1_rx_en = enable;
		span->su_ctrl0.bit.v_b1_tx_en = enable;
	}

	write_xhfc(span->xhfc, R_SU_SEL, span->id);
	write_xhfc(span->xhfc, A_SU_CTRL0, span->su_ctrl0.reg);
	write_xhfc(span->xhfc, A_SU_CTRL2, span->su_ctrl2.reg);

	return 0;
}

static void __init dahdi_chan_init(struct xhfc_chan *chan, int id)
{
	struct xhfc_span *span = chan->span;
	struct xhfc *xhfc = span->xhfc;

	span->chans[id] = &chan->chan;
	chan->chan.pvt = chan;
	
	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	/* Dahdi matches on the B4 to know it is BRI. */
	sprintf(chan->chan.name, "B4/%d/%d/%d",
		xhfc->modidx, span->id + 1, id + 1);

	chan->chan.chanpos = id + 1;
	chan->chan.writechunk = chan->writechunk;
	chan->chan.readchunk = chan->readchunk;

	if (id == 2) { /* d-chan */
		chan->chan.sigcap = DAHDI_SIG_HARDHDLC;
		chan->chan.sig = DAHDI_SIG_HDLCFCS;
	} else {
		chan->chan.sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_DACS;
		chan->chan.sig = DAHDI_SIG_CLEAR;
	}
}

int __init brichannels_create(struct xhfc_span *span)
{
	int ch_index;
	struct xhfc *xhfc = span->xhfc;
	if (debug)
		printk("%s: entered\n", __FUNCTION__);

	/* init B1 channel */
	ch_index = (span->id << 2);
	span->b1_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	bchannel_setup_pcm(&xhfc->chan[ch_index], 0);
	bchannel_setup_pcm(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 0);

	/* init B2 channel */
	ch_index++;
	span->b2_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	bchannel_setup_pcm(&xhfc->chan[ch_index], 0);
	bchannel_setup_pcm(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 1);

	/* init D channel */
	ch_index++;
	span->d_chan = &(xhfc->chan[ch_index]);
	xhfc->chan[ch_index].span = span;
	xhfc->chan[ch_index].id = ch_index;

	dchannel_setup_fifo(&xhfc->chan[ch_index], 0);
	dchannel_setup_fifo(&xhfc->chan[ch_index], 1);

	dahdi_chan_init(&xhfc->chan[ch_index], 2);

	/* Clear PCM  */
	ch_index++;
	memset(&xhfc->chan[ch_index], 0, sizeof(struct xhfc_chan));


	if ( debug)
		printk("brichannels_create exited\n");
	return 0;
}

int __init dchannel_setup_fifo(struct xhfc_chan *chan, unsigned rx)
{
	struct xhfc *xhfc = chan->span->xhfc;
	u8 fifo = (chan->id << 1) + rx;
	
	if (debug)
		printk("dchannel_setup_fifo entered: fifo: %d\n",fifo);
	
	xhfc_selfifo(xhfc, fifo);

	write_xhfc(xhfc, A_CON_HDLC, 0x5);
	write_xhfc(xhfc, A_SUBCH_CFG, 0x2);

	write_xhfc(xhfc, A_FIFO_CTRL,
		   M_FR_ABO | M_FIFO_IRQMSK | M_MIX_IRQ);

	xhfc_resetfifo(xhfc);

	return 0;
}

int __init bchannel_setup_pcm(struct xhfc_chan *chan, unsigned rx)
{
	__u8 fifo = (chan->id << 1) + rx;
#ifdef FPGA_6052
	int timeslot = (chan->id >> 1) + (chan->id & 1);
#else
	int timeslot = (chan->id >> 1) + (chan->id & 1) + 4;
#endif
	struct xhfc *xhfc = chan->span->xhfc;
	/* for module b.. timeslots starts at 12 if old fpga new fpga 32*/
	if (xhfc->modidx == 1)
	{
#ifdef FPGA_6052
		timeslot += 32;
#else
		timeslot += 8;
#endif 
	}
	
	if (debug)
		printk("%s entered - tinmesolt: %d, rx: %d\n", __FUNCTION__,timeslot,rx);


	/* Setup B channel */
	write_xhfc(xhfc, R_SLOT, (timeslot << 1) | rx);
	write_xhfc(xhfc, A_SL_CFG, fifo | 0xC0);
	write_xhfc(xhfc, R_FIFO, fifo);

	write_xhfc(xhfc, A_CON_HDLC, 0xFF);	/* enable fifo for PCM to S/T */

	if (rx == 0) {
		/* tx */
		write_xhfc(xhfc, A_SUBCH_CFG, 0x0); /* 1 start bit, 6 bits */
		write_xhfc(xhfc, A_CH_MSK, 0xFF);
	} else {
		/* rx _SUBCH_CFG MUST ALWAYS BE 000 in RX fifo in
		 * Simple Transparent mode */
		write_xhfc(xhfc, A_SUBCH_CFG, 0);
	}

	/* Reset Fifo*/
	write_xhfc(xhfc, A_INC_RES_FIFO, 0x0A);

	if (debug)
		printk("%s: entering xhfc_waitbusy\n",__FUNCTION__);
	
	xhfc_waitbusy(xhfc);
	if (debug)
		printk("%s: exited xhfc_waitbusy\n",__FUNCTION__);
	
	return 0;
}

static int brie_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < g_brie->num_xhfcs; i++) {
		struct xhfc *xhfc = &g_brie->xhfc[i];
		seq_printf(m, 
				"%c: XHFC-%s R_AF0_OVIEW: %02x  "
				"R_BERT_STA: %02x  "
				"FIFO: rx %u tx %u\n",
				xhfc->modidx ? 'B' : 'A',
				xhfc->num_spans == 4 ? "4SU " : "2S4U",
				xhfc->r_af0_oview, xhfc->r_bert_sta,
				xhfc->rx_fifo_errs, xhfc->tx_fifo_errs);
	}

	return 0;
}

static int brie_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, brie_proc_show, NULL);
}

static const struct file_operations brie_proc_fops = {
	.owner   	= THIS_MODULE,
	.open		= brie_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __init brie_enable_interrupts(struct brie *dev)
{
	struct xhfc *xhfc;
	int i;

	printk("brie_enable_interrupts entering\n");
	for (xhfc = dev->xhfc, i = 0; i < dev->num_xhfcs; i++, ++xhfc) {
		printk("brie_enable_interrupts loop in %d\n",i);
		write_xhfc(xhfc, R_SU_IRQMSK, xhfc->su_irqmask.reg);

		/* clear all pending interrupts bits */
		read_xhfc(xhfc, R_MISC_IRQ);
		read_xhfc(xhfc, R_SU_IRQ);
		read_xhfc(xhfc, R_FIFO_BL0_IRQ);
		read_xhfc(xhfc, R_FIFO_BL1_IRQ);
		read_xhfc(xhfc, R_FIFO_BL2_IRQ);
		read_xhfc(xhfc, R_FIFO_BL3_IRQ);

		/* enable global interrupts */
		xhfc->irq_ctrl.bit.v_glob_irq_en = 1;
		xhfc->irq_ctrl.bit.v_fifo_irq_en = 1;
		write_xhfc(xhfc, R_IRQ_CTRL, xhfc->irq_ctrl.reg);
	}
}

void bri_post_failure(char* err_str)
{
#ifdef CONFIG_POST
    post_fail(POST_BRI_DVR_ID, err_str);
#endif
    printk(KERN_ERR "%s\n", err_str);	
}

/* This must be called *after* setting the PCM master or the FXS/FXO
 * will fail.
 */
static void __init brie_init(struct brie *dev)
{
	unsigned reg;

	if (debug)
		printk("brie_init entering\n");
	reg = fpga_read(dev->fpga, FPGA_CONFIG);
	reg &= ~6; /* clear syncsrc */
	reg |= 0x78; /* set all TE */
	fpga_write(dev->fpga, FPGA_CONFIG, reg);

	/* We always want 100 ohm termination. We have to be careful
	 * here not to affect analog modules.
	 */
	reg = fpga_read(dev->fpga, FPGA_PFT);
	if (dev->moda)
		reg &= ~1;
	if (dev->modb)
		reg &= ~2;
	fpga_write(dev->fpga, FPGA_PFT, reg);
}

static int __init bri_module_init(struct xhfc *xhfc)
{
	reg_r_pcm_md0 pcm_md0;
	reg_r_pcm_md1 pcm_md1;
	u8 threshold;
	int timeout = 0x2000;
	int id = read_xhfc(xhfc, R_CHIP_ID);
	int rev = read_xhfc(xhfc, R_CHIP_RV);

	/* We no longer check the revision (MI#6993). We do not know
	 * if we can work with anthing other than rev 0. */
	if (id == CHIP_ID_2S4U) {
		printk(KERN_INFO "XHFC-2S4U Rev %x\n", rev);
		xhfc->num_spans = 2;
		xhfc->su_irqmask.bit.v_su0_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su1_irqmsk = 1;
	} else if (id == CHIP_ID_4SU) {
		printk(KERN_INFO "XHFC-4SU Rev %x\n", rev);
		xhfc->num_spans = 4;
		xhfc->su_irqmask.bit.v_su0_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su1_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su2_irqmsk = 1;
		xhfc->su_irqmask.bit.v_su3_irqmsk = 1;
	} else {
		printk(KERN_WARNING "Unexpect id %x rev %x (0)\n", id, rev);
		return -ENODEV;
	}

	/* Setup FIFOs */
	if (xhfc->num_spans == 4) {
		/* 64 byte fifos */
		xhfc->max_z = 0x3F;
		xhfc->max_fifo = 16;
		write_xhfc(xhfc, R_FIFO_MD, 0);
		threshold = 0x34;
	} else {
		/* 128 byte fifos */
		xhfc->max_z = 0x7F;
		xhfc->max_fifo = 8;
		write_xhfc(xhfc, R_FIFO_MD, 1);
		threshold = 0x68;
	}

	/* software reset to enable R_FIFO_MD setting */
	write_xhfc(xhfc, R_CIRM, M_SRES);
	udelay(5);
	write_xhfc(xhfc, R_CIRM, 0);

	/* Set GPIO - this enables GPIO pins 0,1,2,3, and 7 then sets
	 * them to be outputs
	 */
	write_xhfc(xhfc, R_GPIO_SEL, 0x8f);
	write_xhfc(xhfc, R_GPIO_EN0, 0x8f);
	write_xhfc(xhfc, R_GPIO_OUT0, 0);

	if (debug)
		printk("%s: Make sure R_STATUS is not busy\n",__FUNCTION__);

	while ((read_xhfc(xhfc, R_STATUS) & (M_BUSY | M_PCM_INIT)) && timeout)
		timeout--;

	if (!timeout) {
		printk(KERN_ERR
		       "%s: initialization sequence could not finish\n",
		       __func__);
		return -ENODEV;
	}

	if (debug)
		printk("%s: Setting threshold\n", __FUNCTION__);

	/* Set threshold *after* software reset done */
	write_xhfc(xhfc, R_FIFO_THRES, threshold);


	if (debug)
		printk("%s: Setting PCM slave mode\n", __FUNCTION__);

	/* set PCM master mode */
	pcm_md0.reg = 0;
	pcm_md1.reg = 0;
	pcm_md0.bit.v_pcm_md = 1; // Master
	write_xhfc(xhfc, R_PCM_MD0, pcm_md0.reg);

	if (debug)
		printk("%s: Adjust the PLL\n", __FUNCTION__);

	/* set pll adjust */
	pcm_md0.bit.v_pcm_idx = IDX_PCM_MD1;
	pcm_md1.bit.v_pll_adj = 3;

	printk("PCM_MD0: %x - pcm_md1: %x\n", pcm_md0.reg,pcm_md1.reg);

	write_xhfc(xhfc, R_PCM_MD0, pcm_md0.reg);
	write_xhfc(xhfc, R_PCM_MD1, pcm_md1.reg);


	spin_lock_init(&xhfc->fifo_tx_lock);
	spin_lock_init(&xhfc->fifo_lock);
	spin_lock_init(&xhfc->dev->tasklet_lock);
	spin_lock_init(&xhfc->dev->workqueue_lock);

	/* DM - Original driver does IRQ test here.. not bothering */

	if (debug)
		printk("%s: exiting with status code 0\n", __FUNCTION__);


	return 0;
}

static int __init bri_module_create(struct brie *dev, int id, int chan)
{
	int err, i;
	struct xhfc *xhfc = &(dev->xhfc[id]);
	char err_str[256];

	xhfc->dev = dev;

#ifdef FPGA_6052
	xhfc->modidx = chan == 0 ? 0 : 1;
#else
	xhfc->modidx = chan == 2 ? 0 : 1;
#endif
	xhfc->chipnum = chan;

	err = bri_module_init(xhfc);
	if (err)
		return err;

	/* alloc mem for all channels */
	xhfc->chan = kzalloc(sizeof(struct xhfc_chan) * MAX_CHAN, GFP_KERNEL);
	if (!xhfc->chan) {
		sprintf(err_str, "%d %s: No kmem for xhfc_chan_t*%i [#500]",
			xhfc->modidx, __func__, MAX_CHAN);
		bri_post_failure(err_str);
		return -ENOMEM;
	}

#ifdef DAHDI_2_6_0
	xhfc->ddev = dahdi_create_device();
	if (!xhfc->ddev)
	{    
		sprintf(err_str, "%s() : cannot create dahdi_device [#501]",   __FUNCTION__);
        	bri_post_failure(err_str);
		return -ENOMEM;
	}    
	xhfc->ddev->location = kasprintf(GFP_KERNEL, "PIKA BRI on module %c", xhfc->modidx  + 'A' );
	if (!xhfc->ddev->location) {
		dahdi_free_device(xhfc->ddev);
		xhfc->ddev = NULL;
		sprintf(err_str, "%s() : cannot create location for module %d [#502]",   __FUNCTION__, xhfc->modidx);
        	bri_post_failure(err_str);
		return -ENOMEM;
	}    

	xhfc->ddev->manufacturer = "Pika";
	if (xhfc->modidx == 0 )
		xhfc->ddev->devicetype = "BRI Module A";
	else
		xhfc->ddev->devicetype = "BRI Module B";

	xhfc->ddev->hardware_id = kasprintf(GFP_KERNEL, "%d", xhfc->modidx);
	if (!xhfc->ddev->hardware_id){
		dahdi_free_device(xhfc->ddev);
		kfree(xhfc->ddev->location);
		xhfc->ddev = NULL;
		//TODO: Fix Post
		sprintf(err_str, "%s() : cannot create hardware_id for module %d [#502]",  __FUNCTION__, xhfc->modidx);
		bri_post_failure(err_str);
		return -ENOMEM;
	}

#endif

	for (i = 0; i < xhfc->num_spans; i++) {
		err = brispan_create(xhfc, i);
		if (err) {
			brispan_cleanup_spans(xhfc);
			kfree(xhfc->chan);
			return err;
		}
	}

	return 0;
}

int __init brie_create_xhfc(struct brie *dev)
{
	int err;
	unsigned imr;
	unsigned id = 0;
//	int syncsrc;
	//P2: unsigned reg = fpga_read(dev->fpga, FPGA_CONFIG);

	// Determine numer of framers
	dev->num_xhfcs = (dev->moda && dev->modb) ? 2 : 1;

	dev->xhfc = kzalloc(sizeof(struct xhfc) * dev->num_xhfcs, GFP_KERNEL);
	if (!dev->xhfc) {
		bri_post_failure("brie_create_xhfc: NO MEMORY [#503]");
		return -ENOMEM;
	}

	/* Enable BRI ints in FPGA before initializing modules. */
	imr = fpga_read(dev->fpga, FPGA_IMR);
	if (dev->moda) {
		imr |= 1 << 20;
//		syncsrc = 2 << 0;
	}
	if (dev->modb) {
		imr |= 1 << 21;
//		syncsrc = 2 << 1;

	}
	
	fpga_write(dev->fpga, FPGA_IMR, imr);

/* P2
	reg = (reg & ~6) | syncsrc;
	fpga_write(dev->fpga, FPGA_CONFIG, reg);
*/


	if (dev->moda) {
		printk(KERN_INFO "  BRI in module A\n");
#ifdef FPGA_6052
		err = bri_module_create(dev, id, 0);
#else
		err = bri_module_create(dev, id, 2);
#endif
		id++;
		if (err)
			goto error_cleanup;
	}
	if (dev->modb) {
		printk(KERN_INFO "  BRI in module B\n");
#ifdef FPGA_6052
		err = bri_module_create(dev, id, 8);
#else
		err = bri_module_create(dev, id, 6);
#endif
		if (err)
			goto error_cleanup;
	}

	/* initial clock config */
	set_clock(dev);

	if (debug)
		printk("brie_create_xhfc exiting\n");

	return 0;

error_cleanup:
	kfree(dev->xhfc);
	return err;
}


int __init brie_create(struct brie *dev)
{
	int rc;

	rc = brie_create_xhfc(dev);
	if (rc)
		return rc;

	
	brie_init(dev);

	return 0;
}

static int __init brie_init_module(void)
{
	int i, err = -EINVAL, pref = 1;
	unsigned rev;
	struct proc_dir_entry *entry;
        char err_str[256];
#ifdef WARP_V2
	struct device_node *np;
#endif

#ifndef WARP_V2
	// Check FPGA is active
	if (warp_fpga_isactive() == 0)
	{    
		printk(KERN_ERR "%s: FPGA is not active\n", __FUNCTION__);
		return -ENODEV;
	}    

	// Init Workqueue
	brie_wq = alloc_workqueue("pika_brie_wq", 0,0);
	if (!brie_wq)
	{
		printk(KERN_ERR __FILE__ " File to allocate workqueue\n");
		return -ENOMEM;
	}
#endif

	/* Create the private data */
	g_brie = kzalloc(sizeof(struct brie), GFP_KERNEL);
	if (!g_brie) {
		printk(KERN_ERR __FILE__ " NO MEMORY\n");
		return -ENOMEM;
	}
	memset(g_brie, 0, sizeof(struct brie));
	g_brie->pcm_master = -2;

#ifdef WARP_V2
	np = of_find_compatible_node(NULL, NULL, "pika,fpga");
	if (!np) {
		printk(KERN_ERR __FILE__ ": Unable to find fpga\n");
		goto error_cleanup;
	}

	g_brie->irq = irq_of_parse_and_map(np, 0);
	if (g_brie->irq  == NO_IRQ) {
		printk(KERN_ERR __FILE__ ": irq_of_parse_and_map failed\n");
		goto error_cleanup;
	}

	g_brie->fpga = of_iomap(np, 0);
	if (!g_brie->fpga) {
		printk(KERN_ERR __FILE__ ": Unable to get FPGA address\n");
		goto error_cleanup;
	}

	of_node_put(np);
	np = NULL;
#else
	g_brie->dma = warp_fpga_getdevice();
	// Keep back warps settings
	g_brie->fpga = g_brie->dma->base;
	g_brie->irq  = g_brie->dma->irq;
#endif

#ifdef USE_TASKLET
	tasklet_init(&g_brie->brie_bh, brie_tasklet, (unsigned long)g_brie);
#else
	INIT_WORK(&g_brie->brie_work, brie_worker);
#endif
	rev = fpga_read(g_brie->fpga, FPGA_REV);
	g_brie->moda = (rev & 0x0f0000) == 0x030000;
	g_brie->modb = (rev & 0xf00000) == 0x300000;

	if (!g_brie->moda && !g_brie->modb) {
		err = -ENODEV;
		goto error_cleanup;
	}

	// Get TX/RX Buffers
	pikadma_get_buffers(&g_brie->rx_buf, &g_brie->tx_buf);

	printk(KERN_INFO "pikabrie starting...\n");

	/* GSM uses the same bits as BRI */
	if (g_brie->moda)
		bri_int_mask |= 1 << 20;
	if (g_brie->modb)
		bri_int_mask |= 1 << 21;

	err = request_irq(g_brie->irq, brie_isr, IRQF_SHARED,"pikabrie", g_brie);
	if (err) {
		sprintf(err_str, "%s : Unable to request irq %d [#504]", __FILE__, err);
		bri_post_failure(err_str);
		goto error_cleanup;
	}

	err = brie_create(g_brie);
	if (debug)
		printk("brie_create: return with: %d\n", err);

	if (err){
		sprintf(err_str, "brie_create: failed with err: %d [#505]", err);
		bri_post_failure(err_str);
		goto irq_cleanup;
	}

	entry = proc_create("driver/bri", 0, NULL, &brie_proc_fops);
	if (!entry) {
		printk(KERN_ERR "Unable to register /proc/driver/bri\n");
		err = -ENOENT;
		goto irq_cleanup;
	}

	printk("Starting Dahdi Registeration\n");
	/* Register with dahdi - do this last */
	for (i = 0; i < MAX_SPANS; ++i) {
		if (g_brie->spans[i]) {
			int card_id = g_brie->spans[i]->modidx;

			if ( card_id == 1 && g_brie->num_xhfcs == 1 )
				card_id--;

			pr_info("i: %d card_id: %d - %s\n", i, card_id, g_brie->spans[i]->span.name);
#ifdef DAHDI_2_6_0			
			list_add_tail(&g_brie->spans[i]->span.device_node, &g_brie->xhfc[card_id].ddev->spans);

#else
			if (dahdi_register(&g_brie->spans[i]->span, pref)) {
				sprintf(err_str, "Unable to register span %s [#507]",
					g_brie->spans[i]->span.name);
				bri_post_failure(err_str);
				goto register_cleanup;
			}
#endif
			pref = 0;
		}
	}

	for (i = 0; i < g_brie->num_xhfcs; i++) {
		printk("Starting dahdi_register_device: %d\n", i);
		if (dahdi_register_device(g_brie->xhfc[i].ddev, &g_brie->dma->pdev->dev)) {
			sprintf(err_str, "Unable to register span with DAHDI %d [#508]", i);
			bri_post_failure(err_str);
			kfree(g_brie->xhfc[i].ddev->location);
			dahdi_free_device(g_brie->xhfc[i].ddev);
			g_brie->xhfc[i].ddev = NULL;
			goto register_cleanup;
		}
	}

	/* And start it */
	brie_enable_interrupts(g_brie);

	err = brie_dma_enable(g_brie);
	if (err)
	{
		sprintf(err_str, "error: %d in brie_dma_enable [#509]", err);
		bri_post_failure(err_str);
		goto register_cleanup;
	}
         
#ifdef CONFIG_POST
    post_pass(POST_BRI_DVR_ID, "BRI Module - initialized successfully");
#endif
	return 0;

register_cleanup:

	for (i = 0; i < g_brie->num_xhfcs; i++) {
		dahdi_unregister_device(g_brie->xhfc[i].ddev);
		kfree(g_brie->xhfc[i].ddev->location);
		dahdi_free_device(g_brie->xhfc[i].ddev);
	}

/* TODO
	remove_proc_entry("driver/bri", NULL);
*/
irq_cleanup:
	if ( g_brie->irq )
		free_irq(g_brie->irq, g_brie);

error_cleanup:
#ifdef WARP_V2
	if (np)
		of_node_put(np);
	if (g_brie->fpga)
		iounmap(g_brie->fpga);
#else
	g_brie->fpga = NULL;
	g_brie->dma = NULL;
#endif
	destroy_workqueue(brie_wq);

	kfree(g_brie);

	return err;
}
module_init(brie_init_module);

static void __exit brie_destroy_xhfc(struct brie *dev)
{
	struct xhfc *xhfc;
	int i;

	if (!dev->xhfc)
		return;

	flush_workqueue(brie_wq);

	for (i = 0; i < MAX_SPANS; ++i) {
		if ( dev->spans[i] ) {
			brispan_destroy(dev->spans[i]);
			dev->spans[i] = NULL;
		}
	}

	for (i = 0, xhfc = dev->xhfc; i < dev->num_xhfcs; i++, ++xhfc)
		kfree(xhfc->chan);

	kfree(dev->xhfc);
}

void __exit brie_exit_module(void)
{
	int i;

	/* Disable BRI ints */
	unsigned imr = fpga_read(g_brie->fpga, FPGA_IMR);
	imr &= ~bri_int_mask;
	fpga_write(g_brie->fpga, FPGA_IMR, imr);

	brie_stop(g_brie);

	for (i = 0; i < g_brie->num_xhfcs; i++) {
		dahdi_unregister_device(g_brie->xhfc[i].ddev);
		kfree(g_brie->xhfc[i].ddev->location);
		dahdi_free_device(g_brie->xhfc[i].ddev);
	}


	brie_destroy_xhfc(g_brie);
	if (g_brie->irq)
		free_irq(g_brie->irq, g_brie);

	remove_proc_entry("driver/bri", NULL);
#ifdef WARP_V2
	iounmap(g_brie->fpga);
#else
	g_brie->fpga = NULL;
	g_brie->dma =NULL;
#endif
	
	destroy_workqueue(brie_wq);

	kfree(g_brie);
}
module_exit(brie_exit_module);

MODULE_DESCRIPTION("PIKA BRI Driver");
MODULE_AUTHOR("Pawel Pastuszak");
MODULE_LICENSE("GPL");
