#include <linux/module.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <asm/time.h>
#include <asm/io.h>

#include <dahdi/kernel.h>

#include <pika/driver.h>
#include <pika/daytona.h>
#include <pika/wa_parms.h>
#include "wa_devext.h"
#include <pika/wa_dma.h>

int pft_init(PDEVICE_EXTENSION pdx);

/* dma initialization function */
int warp_dma_init(PDEVICE_EXTENSION pdx)
{
	pikadma_get_buffers(&pdx->rx_buf, &pdx->tx_buf);
	return 0;
}

/* dma finalization/teardown function */
void warp_dma_fini(PDEVICE_EXTENSION pdx) 
{

}

/* dma enable control function -- TODO : call the function directly */
#ifdef WARP_V2
int warp_dma_enable(void (*dma_handler)(int))
#else
int warp_dma_enable(void (*dma_handler)(int,void * arg))
#endif
{
	return warp_dma_register(dma_handler);
}

/* dma disable control function -- TODO : call the function directly */
void warp_dma_disable(void (*dma_handler)(int,void * arg))
{
	warp_dma_unregister();
}

/* This is *only* called once at module init time! */
int warp_pft_init(PDEVICE_EXTENSION pdx)
{
	return pft_init(pdx);
}

/*
 * Force kernel coding standard on PIKA code
 * Local Variables:
 * tab-width: 8
 * c-basic-offset: 8
 * indent-tabs-mode: t
 * End:
 */
