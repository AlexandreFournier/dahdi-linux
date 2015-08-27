#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#ifdef CONFIG_POST
#include <linux/post.h>
#endif

#include <pika/driver.h>
#include <pika/warp-fpga.h>

extern int debug;

struct dma_inst {
	struct list_head list;
	int cardid;
	void * arg;
	void (*cb)(int cardid, void *arg);
};

struct dsp_inst {
	void * arg;
	void (*dsp_isr)(int dspid, void *arg);
};

static struct dsp_inst dsp_device;

#define MODNAME 				"pikadma"
#define DMA_INTS				0xC0000

// Misc DMA Post Test globals
#define FRAMES_PER_BUFFER			(40 * 8) // 320 = 40 ms
#define FRAMES_PER_TRANSFER			8
#define FRAMESIZE				64

#define JITTER					2
#define DMA_BUFFER_SIZE                         FRAMES_PER_BUFFER * FRAMESIZE 
#define ENTRIES                                 FRAMES_PER_BUFFER / FRAMES_PER_TRANSFER
#define LOOPBACK_TESTS				16

// There is only one instance of the dma 
static struct warp_fpga * g_dma_ctx;

int warp_fpga_set_dsp_callback(void (*f)(int dspid,void *arg), void * arg)
{
	dsp_device.dsp_isr = f;
	dsp_device.arg = arg;
	return 0;
}
EXPORT_SYMBOL(warp_fpga_set_dsp_callback);

static irqreturn_t dma_isr(int irq, void *context)
{
	struct dma_inst *inst;
	unsigned iccr = fpga_read(g_dma_ctx->fpga, FPGA_ICCR) & DMA_INTS;

	if (!iccr)
		return IRQ_NONE;

	fpga_write(g_dma_ctx->fpga, FPGA_ICCR, iccr);
	// printk(KERN_INFO "interrupt detected! %08x\n", iccr);

#ifdef DMA_DISABLED
	printk(KERN_INFO "DMA interrupt in disabled state!\n");
	return IRQ_HANDLED;
#endif

	if (list_empty(&g_dma_ctx->list))
		return IRQ_HANDLED;
	
	if (iccr & 0x40000)
		printk(KERN_ERR "DMA underrun\n");

	if (iccr & FPGA_ICCR_DMA_DONE_MASK) {
		// DMA Good
		list_for_each_entry(inst, &g_dma_ctx->list, list)
			inst->cb(inst->cardid, inst->arg);
	}

	return IRQ_HANDLED;
}

static struct dma_inst *find_inst(struct warp_fpga *dma, int cardid)
{
	struct dma_inst *inst;

	list_for_each_entry(inst, &dma->list, list)
		if (inst->cardid == cardid)
			return inst;

	return NULL;
}

int pikadma_register_cb(int cardid, void (*cb)(int cardid,void * arg), void * arg)
{
	struct warp_fpga *dma = g_dma_ctx;
	struct dma_inst *inst;
	unsigned imr = fpga_read(dma->fpga, FPGA_IMR);

	if (cardid == -1)
		return -EINVAL;

	if (find_inst(dma, cardid)) {
		printk(KERN_WARNING "dma_register: cardid %d already registered.\n", cardid);
		return -EBUSY;
	}

	inst = kzalloc(sizeof(struct dma_inst), GFP_KERNEL);
	if (!inst) {
		printk(KERN_ERR "dma_register: out of memory\n");
		return -ENOMEM;
	}

	// Disable the interrupts while adding to list.
	fpga_write(dma->fpga, FPGA_IMR, imr & ~DMA_INTS);

	inst->cardid = cardid;
	inst->arg = arg;
	inst->cb = cb;
	list_add_tail(&inst->list, &dma->list);

#ifndef DMA_DISABLED
	/* Always safe to reenable */
	fpga_write(dma->fpga, FPGA_IMR, imr | DMA_INTS);
	fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 1);
#endif

	return 0;
}
EXPORT_SYMBOL(pikadma_register_cb);

int pikadma_unregister(int cardid)
{
	struct warp_fpga *dma = g_dma_ctx;
	struct dma_inst *inst = find_inst(dma, cardid);
	unsigned imr = fpga_read(dma->fpga, FPGA_IMR);

	if (!inst)
		return -EINVAL;

	// Disable ints while removing from list
	fpga_write(dma->fpga, FPGA_IMR, imr & ~DMA_INTS);
	list_del(&inst->list);
	kfree(inst);

#ifndef DMA_DISABLED
	if (list_empty(&dma->list))
		fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 0);
	else /* reenable ints */
		fpga_write(dma->fpga, FPGA_IMR, imr);
#endif

	return 0;
}
EXPORT_SYMBOL(pikadma_unregister);

void pikadma_get_buffers(u8 **rx_buf, u8 **tx_buf)
{
	struct warp_fpga *dma = g_dma_ctx;

	if (rx_buf)
		*rx_buf = dma->rx_buf;
	if (tx_buf)
		*tx_buf = dma->tx_buf;
}
EXPORT_SYMBOL(pikadma_get_buffers);

void pikadma_disable(void)
{
	fpga_write(g_dma_ctx->fpga, FPGA_TDM_DMA_CTRL, 0);
}
EXPORT_SYMBOL(pikadma_disable);

void pikadma_enable(void)
{
	fpga_write(g_dma_ctx->fpga, FPGA_TDM_DMA_CTRL, 1);
}
EXPORT_SYMBOL(pikadma_enable);

int create_scatter_gather(struct warp_fpga *dma, int entries, int test_mode)
{
	unsigned rx_buf, tx_buf, sgl = 0;
	int bytes = FRAMES_PER_TRANSFER * FRAMESIZE;
	int i;

	dma->dma_size = DMA_BUFFER_SIZE * 2;
	dma->dma_buf = dma_alloc_coherent(dma->dev, dma->dma_size, &dma->dma_handle, GFP_DMA);
	if (!dma->dma_buf)
		return -ENOMEM;

	dma->rx_buf = dma->dma_buf;
	dma->tx_buf = dma->dma_buf + DMA_BUFFER_SIZE;
	
	rx_buf = dma->dma_handle;
	tx_buf = dma->dma_handle + DMA_BUFFER_SIZE;

	//if (debug) {
		printk(KERN_INFO "DMA HANDLE %x\n", dma->dma_handle);
		printk(KERN_INFO "DMA RX buffer %p physical %p\n", dma->rx_buf, rx_buf);
		printk(KERN_INFO "DMA TX buffer %p physical %p\n", dma->tx_buf, tx_buf);
	//}
	
	if (test_mode == 0)
		tx_buf |= 2; /* HSP requires us to interrupt on each frame pair */

	printk(KERN_INFO MODNAME ": Add scatter-gather list %d entries\n", entries);
	for (i = 0; i < entries; ++i)
	{
		if (test_mode == 1) {
			if ((entries / 2) - 1 == i)
				tx_buf |= 2;
			else
				tx_buf &= ~ 2;
		}

		if (i == entries - 1)
		{
			if (test_mode)
				tx_buf |= 3; /* wrap bit */
			else
				tx_buf |= 1; /* wrap bit */
		}

		fpga_write(dma->sgl, sgl, rx_buf);
		sgl += 4;

		fpga_write(dma->sgl, sgl, tx_buf);
		sgl += 4;

		rx_buf += bytes;
		tx_buf += bytes;
	}

	// Program transfer size to FRAMES_PER_TRANSFER frames, and jitter to JITTER
	printk(KERN_INFO MODNAME ": Set up DMA sizes\n");
	fpga_write(dma->fpga, FPGA_TDM_DMA_SZ,  FRAMES_PER_TRANSFER);
	fpga_write(dma->fpga, FPGA_TDM_DMA_JIT, JITTER);

	return 0;
}
EXPORT_SYMBOL(create_scatter_gather);

void destroy_scatter_gather(struct warp_fpga *dma, int entries)
{
	int i = 0, sgl = 0;

	if (dma->dma_buf)
		dma_free_coherent(dma->dev, dma->dma_size, dma->dma_buf, dma->dma_handle);

	dma->dma_buf = NULL;
	dma->rx_buf  = NULL;
	dma->tx_buf  = NULL;
	
	for (i = 0; i < entries; ++i) {
		fpga_write(dma->sgl, sgl, 0);
		sgl += 4;

		fpga_write(dma->sgl, sgl, 0);
		sgl += 4;
	}
}
EXPORT_SYMBOL(destroy_scatter_gather);

#ifdef FPGA_6052
int dma_loop_back_test(void)
{
	struct warp_fpga *dma = g_dma_ctx;
	u8 *rx_buffer_ptr;
	u8 *tx_buffer_ptr;
	unsigned long tx_frame_counter;
	unsigned long rx_frame_counter;
	uint ram_index;
	uint skip_counter;
	uint loop_count;
	uint random_character_counter;
        char post_log[500];
	int timeout;
	int i;

	// Testing SGL & DMA with TDM-loopback
	printk(KERN_INFO MODNAME ": Testing DMA loopback for SGL\n");

	// Set up TDM loopback in the FPGA
	printk(KERN_INFO MODNAME ": Turn on PCM loopback\n");
 	fpga_write(dma->fpga, FPGA_DIAG, fpga_read(dma->fpga, FPGA_DIAG) | FPGA_PCM_LOOPBACK_BIT);

 	// Turn off DMA if it's on
	printk(KERN_INFO MODNAME ": Turn off DMA\n");
 	fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 0);

	// Create scatter-gather list for testing
	if (create_scatter_gather(dma, ENTRIES, 1)) {
		printk(KERN_ERR MODNAME ": Unable to allocate SGL\n");
		goto error_cleanup;
	}

	// Update local pointers to DMA buffers
	rx_buffer_ptr = dma->rx_buf;
	tx_buffer_ptr = dma->tx_buf;

	// Refresh the rx and tx buffer
	for (ram_index = 0; ram_index < DMA_BUFFER_SIZE; ram_index++) {
		// The RX pattern should get overwritten 
		rx_buffer_ptr[ram_index] = 0xAD;

		// The TX pattern encodes the frame number
		if ((ram_index % FRAMESIZE) == 0) {
			tx_buffer_ptr[ram_index] = 0x00;
			tx_frame_counter++;
		} else if ((ram_index % FRAMESIZE) == 1) {
			tx_buffer_ptr[ram_index] = (u8)(tx_frame_counter >> 8);
		} else if ((ram_index % FRAMESIZE) == 2) {
			tx_buffer_ptr[ram_index] = (u8)(tx_frame_counter & 0xFF);
		} else {
			tx_buffer_ptr[ram_index] = (u8)(((ram_index % FRAMESIZE) * 13) + tx_frame_counter);
		}
	}

	// Turn on DMA and cycle through the first couple of cycles, since they contain prefill
	printk(KERN_INFO MODNAME ": Turn on DMA\n");
	fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 1);
	fpga_write(dma->fpga, FPGA_ICCR, FPGA_ICCR_DMA_DONE_MASK); // Clear done bit
	
	// DMA Interrupt Test
	printk(KERN_INFO MODNAME ": DMA Interrupt Test\n");
	for (i = 0; i < 5; i++)
	{
		timeout = 0;
		while((fpga_read(dma->fpga, FPGA_ICCR) & FPGA_ICCR_DMA_DONE_MASK) == 0)
		{
			udelay(1000); // delay 1ms 
			timeout++;
			if (timeout == 5)
			{
				printk(KERN_ERR MODNAME ": No TDM input clock to FPGA.\n");
#ifdef CONFIG_POST
				post_fail(POST_DMA_DVR_ID, "No TDM input clock to FPGA. [#601]");
#endif
				goto error_cleanup;
			}
		}
		fpga_write(dma->fpga, FPGA_ICCR, 0xFFFFFFFF); // Clear done bit
	}

	// Loopback Test
	printk(KERN_INFO MODNAME ": Loopback Test\n");

	// Loop through and check multiple buffers - pattern should come out
	tx_frame_counter = 0;
	rx_frame_counter = 0;
	skip_counter = 0;
	random_character_counter = 0;	

	for (loop_count = 0; loop_count < LOOPBACK_TESTS; loop_count++) 
	{
		// Debug
		printk(KERN_INFO MODNAME ": Loop %d/%d\n", loop_count + 1, LOOPBACK_TESTS);

		// Wait for the host buffer to be at mid-mark
		timeout = 0;
		while (fpga_read(dma->fpga, FPGA_DMA_SG_IDX) < 4)
		{ 
			udelay(1); // delay 10us
			timeout++;
			if (timeout == 8000)
			{
				printk(KERN_ERR MODNAME ": Cannot find midway-mark of DMA list.\n");
#ifdef CONFIG_POST
				post_fail(POST_DMA_DVR_ID, "Cannot find midway-mark of DMA list. [#602]");
#endif
                                goto error_cleanup;
			}
		}

		// Refresh first half of the tx buffer (just been consumed)
		for (ram_index = 0; ram_index < DMA_BUFFER_SIZE / 2; ram_index++ ) 
		{
			if ((ram_index % FRAMESIZE) == 0) 
			{
				tx_buffer_ptr[ram_index] = 0x00;
				tx_frame_counter++;
			} 
			else if ((ram_index % FRAMESIZE) == 1) 
			{	
				tx_buffer_ptr[ram_index] = (u8)(tx_frame_counter >> 8);
			} 
			else if ((ram_index % FRAMESIZE) == 2)
			{	
				tx_buffer_ptr[ram_index] = (u8)tx_frame_counter & 0xFF;
			} 
			else 
			{
				tx_buffer_ptr[ram_index] = (u8)(((ram_index % FRAMESIZE) * 13) + tx_frame_counter);
			}
		}

		// Verify contents of the first half of the rx buffer for corruption
		for (ram_index = 0; ram_index < DMA_BUFFER_SIZE / 2; ram_index++) 
		{
			if ((ram_index % FRAMESIZE) == 0) 
			{
				if (rx_buffer_ptr[ram_index] != 0x00)
				{
					// Slot 0 does not contain zero - it is corrupt
					
					sprintf(post_log,"Slot 0 corrupt with value 0x%X (should have been 0). [#603]", rx_buffer_ptr[ram_index]); 
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
					post_fail(POST_DMA_DVR_ID, post_log);
#endif

					sprintf(post_log, "Failure at ram index 0x%X loop_count %d [#604]", ram_index, loop_count);
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
					post_fail(POST_DMA_DVR_ID, post_log);
#endif

					goto error_cleanup;
				} 
				else 
				{
					rx_frame_counter++;
				}
			} 
			else if ((ram_index % FRAMESIZE) == 1 && rx_buffer_ptr[ram_index] != (u8)(rx_frame_counter >> 8)) 
			{
				// Our frame counter msb is incorrect - this could be due to unavailability of our app
				// to get CPU bandwidth.  Increment skip counter and realign frame counter
				rx_frame_counter = (rx_buffer_ptr[ram_index] << 8) | (rx_frame_counter & 0xFF);
				skip_counter++;
			} 
			else if ((ram_index % FRAMESIZE) == 2 && rx_buffer_ptr[ram_index] != (u8)(rx_frame_counter & 0xFF)) 
			{
				// Our frame counter lsb is incorrect - this could be due to unavailability of ou app
				// to get CPU bandwidth.  Increment skip counter and realign frame counter
				rx_frame_counter = (rx_frame_counter & 0xFF00) | rx_buffer_ptr[ram_index];
				skip_counter++;
			}
			else
			{
				random_character_counter = random_character_counter + 1;
				if ((ram_index % FRAMESIZE) > 2 && rx_buffer_ptr[ram_index] != (u8)((ram_index % FRAMESIZE * 13) + rx_frame_counter)) 
				{
					// The slot contents are corrupt - they should correspond to a sequence based on the frame counter
	
					// Stop DMA
					fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 0);

					sprintf(post_log, "Slot %d (0-63) corrupt with value 0x%X, expected 0x%X. rx_frame_counter = %d [#605]", 
						ram_index % FRAMESIZE, rx_buffer_ptr[ram_index],
				   		(u8) (((ram_index % FRAMESIZE) * 13) + rx_frame_counter) ); 
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
					post_fail(POST_DMA_DVR_ID, post_log);
#endif

					sprintf(post_log, "Failure at ram index 0x%X loop_count %d [#606]", ram_index, loop_count);
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
					post_fail(POST_DMA_DVR_ID, post_log);
#endif
				}
			}
			// End if selected timeslot
		}

		// Wait for the host buffer to be back at the beginning
		timeout = 0;
		while (fpga_read(dma->fpga, FPGA_DMA_SG_IDX) >= 4) 
		{
			udelay(1); // delay 10us
			timeout++;
			if (timeout == 8000)
			{
				printk(KERN_ERR MODNAME ": Host buffer never returns to the beginning.\n");
#ifdef CONFIG_POST
				post_fail(POST_DMA_DVR_ID, "Host buffer never returns to the beginning. [#607]");
#endif
				goto error_cleanup;
			}		
		}

		// Refresh last half of the transmit buffer (just been consumed)
		for (ram_index = DMA_BUFFER_SIZE / 2; ram_index < DMA_BUFFER_SIZE; ram_index++)
		{
			if ((ram_index % FRAMESIZE) == 0)
			{
				// timeslot 0 = 0
				tx_buffer_ptr[ram_index] = 0x00;
				tx_frame_counter++;
			}	
			else if ((ram_index % FRAMESIZE) == 1) 
			{	
				tx_buffer_ptr[ram_index] = (u8)(tx_frame_counter >> 8);
			}
			else if ((ram_index % FRAMESIZE) == 2) 
			{	
				tx_buffer_ptr[ram_index] = (u8)(tx_frame_counter & 0xFF);
			} 
			else 
			{
				tx_buffer_ptr[ram_index] = (u8)(((ram_index%FRAMESIZE) * 13) + tx_frame_counter);
			}
		}

		// Verify contents of the last half of the rx buffer for corruption
		for (ram_index = DMA_BUFFER_SIZE / 2; ram_index < DMA_BUFFER_SIZE; ram_index++) 
		{
			if ((ram_index % FRAMESIZE) == 0) 
			{
				if (rx_buffer_ptr[ram_index] != 0x00) 
				{
					// Slot 0 does not contain zero - it is corrupt
					fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 0);
					sprintf(post_log, "Slot 0 corrupt with value 0x%X (should have been 0) - loop_count = %d . [#608]", rx_buffer_ptr[ram_index], loop_count); 
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
					post_fail(POST_DMA_DVR_ID, post_log);
#endif
				} 
				else 
				{
					rx_frame_counter++;
				}
			}	
			else if (((ram_index % FRAMESIZE) == 1) && rx_buffer_ptr[ram_index] != (u8)(rx_frame_counter >> 8)) 
			{
				// Our frame counter msb is incorrect - this could be due to unavailability of ou app
				// to get CPU bandwidth.  Increment skip counter and realign frame counter
				rx_frame_counter = (rx_buffer_ptr[ram_index] <<8) | (rx_frame_counter & 0xFF);
				skip_counter++;
			} 
			else if (((ram_index % FRAMESIZE) == 2) && rx_buffer_ptr[ram_index] != (u8)(rx_frame_counter & 0xFF)) 
			{
				
				// Our frame counter lsb is incorrect - this could be due to unavailability of ou app
				// to get CPU bandwidth.  Increment skip counter and realign frame counter
				rx_frame_counter = (rx_frame_counter & 0xFF00) | rx_buffer_ptr[ram_index];
				skip_counter++;
			} 
			else
			{
				random_character_counter = random_character_counter + 1;
				if ( (ram_index % FRAMESIZE) > 2 && rx_buffer_ptr[ram_index] != (u8)(((ram_index%FRAMESIZE) * 13) + rx_frame_counter) ) 
				{
					// The slot contents are corrupt - they should correspond to a sequence based 

		  			sprintf(post_log, "Slot #%d (0-63) corrupt with value 0x%X, expected 0x%X. [#609]",ram_index % FRAMESIZE,
						rx_buffer_ptr[ram_index], ((ram_index%FRAMESIZE * 13) + rx_frame_counter) & 0xFF ); 
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
		  			post_fail(POST_DMA_DVR_ID, post_log);
#endif
		  			sprintf(post_log, "Failure at ram index 0x%X loop_count %d [#610]", ram_index, loop_count);
					printk(KERN_ERR MODNAME ": %s\n", post_log);
#ifdef CONFIG_POST
		  			post_fail(POST_DMA_DVR_ID, post_log);
#endif
					goto error_cleanup;
				}
			}
		} // End FOR ram_index
	}

	// Turn off TDM loopback in the FPGA 
	fpga_write(dma->fpga, FPGA_DIAG, fpga_read(dma->fpga, FPGA_DIAG) & ~FPGA_PCM_LOOPBACK_BIT);

	// Test complete, stop DMA
	fpga_write(dma->fpga, FPGA_TDM_DMA_CTRL, 0);

	printk(KERN_INFO MODNAME ": DMA Initialized successfully and loopback test passed.\n");
#ifdef CONFIG_POST
	post_pass(POST_DMA_DVR_ID, "DMA Initialized successfully and loopback test passed.");
#endif
	return 0;

error_cleanup:

	// Destroy scatter-gather list for testing
	destroy_scatter_gather(dma, ENTRIES);

	// Turn off TDM loopback in the FPGA 
	fpga_write(dma->fpga, FPGA_DIAG, fpga_read(dma->fpga, FPGA_DIAG) & ~FPGA_PCM_LOOPBACK_BIT);

	return 1;
} 
#endif //FPGA_6052

int dma_init_module(struct warp_fpga * dma)
{
	INIT_LIST_HEAD(&dma->list);
	g_dma_ctx = dma;

	// Check FPGA's SGL zone is mapped
	if (!dma->sgl)
	{
		printk(KERN_ERR MODNAME ": Unable to get SGL address\n");
		return -ENOENT;
	}

	// Take straight from device extension
	if (create_scatter_gather(dma, ENTRIES, 0)) {
		printk(KERN_ERR MODNAME ": Unable to allocate SGL\n");
		goto error_cleanup;
	}

	// Disable ALL IRQs ?
	//fpga_write(dma->fpga, FPGA_IMR, 0);

	if (request_irq(dma->irq, dma_isr, IRQF_SHARED, "pikadma", dma)) {
		printk(KERN_ERR MODNAME ": Unable to request irq\n");
		goto error_cleanup;
	}

#ifdef DMA_DISABLED
	printk(KERN_WARNING "Warning: DMA disabled\n");
#endif
	return 0;

error_cleanup:
	destroy_scatter_gather(dma, ENTRIES);
	return -ENOENT;
}
EXPORT_SYMBOL(dma_init_module);

void dma_exit_module(void)
{
	struct warp_fpga *dma = g_dma_ctx;
	
	if (dma->sgl)
		destroy_scatter_gather(dma, ENTRIES);
}
EXPORT_SYMBOL(dma_exit_module);


