/*
 * Power Fail Transfer
 *  Here we handle the logic for FXS/FXO Power Fail transfer mode
 */

#include <pika/pikacore.h>
#ifdef WARP_V2
#include <linux/of_platform.h>
#include <linux/of.h>
#endif
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>

#include <pika/driver.h>
#include <pika/pksystemver.h>


/* We must be very careful with the PFT register since the BRI module
 * reuses the bits for something else :(
 */
static inline void reset_relays(PDEVICE_EXTENSION pdx)
{
	unsigned pft = fpga_read(pdx->bar0, BAR0_TACO_PFT);
	pft &= ~pdx->pft_mask;
	fpga_write(pdx->bar0, BAR0_TACO_PFT, pft);
}

static void clear_line(PDEVICE_EXTENSION pdx, int line)
{
    pdx->disabled_mask &= ~(1 << line);
    add_event(pdx, PFT_EVENT, line, 0); 
}

int pft_thread(void *arg)
{
    PDEVICE_EXTENSION pdx = arg;
    int count_a = 0, count_b = 0;
    unsigned val;

    while (pdx->disabled_mask) {
        if (kthread_should_stop())
            return 0;

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(100));

        val = fpga_read(pdx->bar0, BAR0_TACO_PFT);
#ifdef FPGA_6052
        if (pdx->disabled_mask & (1 << 0)) { /* line 0 */
#else
		if (pdx->disabled_mask & (1 << 2)) { /* line 2 */
#endif
            if (val & 4) {
                if (++count_a > 20) {
                    val |= 1;
                    BAR0_WRITE(pdx, BAR0_TACO_PFT, val);
#ifdef FPGA_6052
                    clear_line(pdx, 0);
#else
					clear_line(pdx, 2);
#endif
                }
            } else
                count_a = 0;
        }
#ifdef FPGA_6052
        if (pdx->disabled_mask & (1 << 8)) { /* line 8 */
#else
		if (pdx->disabled_mask & (1 << 6)) { /* line 6 */
#endif
            if (val & 8) {
                if (++count_b > 20) {
                    val |= 2;
                    fpga_write(pdx->bar0, BAR0_TACO_PFT, val);
#ifdef FPGA_6052
                    clear_line(pdx, 8);
#else
					clear_line(pdx, 6);
#endif
                }
            } else
                count_b = 0;
        }
    }

    pdx->pft = NULL;
    return 0;
}


int pft_init(PDEVICE_EXTENSION pdx)
/* This is *only* called once at module init time! */
{
    struct task_struct *pft;
    unsigned rev = fpga_read(pdx->bar0, BAR0_FPGA_REV) >> 16; 

    pdx->disabled_mask = 0;
    if ((rev & 0x0f) < 0x03)
#ifdef FPGA_6052
        pdx->disabled_mask |= (1 << 0); 
#else
		pdx->disabled_mask |= (1 << 2);
#endif
    if ((rev & 0xf0) < 0x30)
#ifdef FPGA_6052
        pdx->disabled_mask |= (1 << 8); 
#else
		pdx->disabled_mask |= (1 << 6);
#endif

    if (pdx->disabled_mask) {
        reset_relays(pdx); /* paranoia */
		printk("pft_init: Starting Power Fail Transfer Monitoring\n");
        pft = kthread_run(pft_thread, pdx, "pika_pft");
        if (IS_ERR(pft))
            return PTR_ERR(pft);

        pdx->pft = pft;
    }   

    return 0;
}

void pft_stop(PDEVICE_EXTENSION pdx) {

	if ( pdx->pft ) 
	{
		kthread_stop(pdx->pft);
		pdx->pft = NULL;
	}
}

