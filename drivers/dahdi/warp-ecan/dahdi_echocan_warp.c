/* WARP Echo Canceller 
 * (c) 2010 PIKA Technologies Inc.
 * 
 * This program is free software, distributed under the terms of
 * the GNU General Public License version 2 as published by the
 * Free Software Foundation. See the license file included with
 * this program for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/moduleparam.h>

#include <linux/proc_fs.h>

#include <dahdi/kernel.h>
#include <pika/warp-fpga.h>
#include "../warpdahdi-dsp/warp-dsp.h"

#define DAHDI_2_5_0	1

static int echo_can_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			   struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec);
static void echo_can_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);
static void echo_can_process(struct dahdi_echocan_state *ec, short *isig, const short *iref, u32 size);
static int echo_can_traintap(struct dahdi_echocan_state *ec, int pos, short val);
static void echocan_NLP_toggle(struct dahdi_echocan_state *ec, unsigned int enable);

#if defined( DAHDI_2_5_0 )
static const char *warp_ec_name = "WARP";
static const char *ec_name(const struct dahdi_chan *chan) { return warp_ec_name; }

static const struct dahdi_echocan_factory warp_ecan_factory = {
	.get_name = ec_name,
	.owner = THIS_MODULE,
	.echocan_create = echo_can_create,
};

#else 
static const struct dahdi_echocan_factory warp_ecan_factory = {
	.name = "WARP",
	.owner = THIS_MODULE,
	.echocan_create = echo_can_create,
};
#endif

static const struct dahdi_echocan_features warp_ecan_features = {
	.NLP_toggle = 1,
};

static const struct dahdi_echocan_ops warp_ecan_ops = {
#ifndef DAHDI_2_5_0
	.name = "WARP",
#endif
	.echocan_free = echo_can_free,
	.echocan_process = echo_can_process,
	.echocan_traintap = echo_can_traintap,
	.echocan_NLP_toggle = echocan_NLP_toggle,
};

static unsigned moda, modb;

#define dahdi_to_pvt(a) container_of(a, struct warp_ec_pvt, dahdi)
#define module_printk(level, fmt, args...) printk(level "%s: " fmt, THIS_MODULE->name, ## args)

/* Include these function prototypes and the module type definitions in a header file */
void get_module_types(unsigned *moda, unsigned *modb)
{
	unsigned rev = fpga_read(g_dma_ctx->fpga, BAR0_FPGA_REV);

	*moda = (rev >> 16) & 0xf;
	*modb = (rev >> 20) & 0xf;
}


/* Currently supported modules for DAHDI are FXO and FXS only */
#define MODULE_TYPE_FXO         0x01
#define MODULE_TYPE_FXS         0x02
#define MODULE_TYPE_BRI         0x03
#define MODULE_TYPE_GSM         0x04
#define MODULE_TYPE_NONE        0x0F

#define FXS_CHAN_HEADER		"FXS/1/"
#define FXO_CHAN_HEADER 	"FXO/2/"

#define BRI_CHANNEL_FMT 	"B4/%d/%d/%d"
#define BRI_CHANNEL_PARAMCOUNT	3

#define DEFAULT_DSP_ID		0

/* */
struct warp_ec_pvt {
	struct dahdi_echocan_state dahdi;
	int timeslot;
	int dsp_channel;
	int dsp_id;
	/* add other parameters as necessary */
};

/* todo : move this into the proper header file */
int dahdi_warp_get_echocan_channel(u32 dsp_id, u32 in_timeslot, u32 tap_length, u32 *channel_id);
int dahdi_warp_release_echocan_channel(u32 dsp_id, u32 channel_id);

static int
echo_can_get_fxs_timeslot(char *name)
{
	int first_timeslot;
	int res = 0;
	int chan;
	char *sptr;

	/* four possible cases here       */
	/* mod   (A),  nonFXS(B) */
	/* nonFXS(A),  mod  (B)  */
	/* mod   (A),  mod  (B)  */
	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with (%s)\n",
			__FUNCTION__, name);

	if ((sptr = strstr(name, FXS_CHAN_HEADER))) {
		sptr += strlen(FXS_CHAN_HEADER);	
		chan = simple_strtol(sptr, NULL, 10);
	} else {
		printk(KERN_CRIT "%s() : BUG() FXS substring not found.\n",
			__FUNCTION__);
		return (res);
	}

	if  ((moda == MODULE_TYPE_FXS) && (modb != MODULE_TYPE_FXS)) {
		first_timeslot = 2;
	} else if  ((moda != MODULE_TYPE_FXS) && (modb == MODULE_TYPE_FXS)) {
		first_timeslot = 6;
	} else if  ((moda == MODULE_TYPE_FXS) && (modb == MODULE_TYPE_FXS)) {
		first_timeslot = 2;
	} else {
		printk(KERN_ERR "%s() : BUG() Invalid Case\n", __FUNCTION__);
		res = -1;
	}

	if (res == 0) {
		res = ( first_timeslot + chan ) << 1;
		dahdi_warp_log(DSP_ECAN_LOG, "%s() name (%s) => timeslot (%d)\n",
			__FUNCTION__, name, res);
	}

	return (res);
}

static int
echo_can_get_fxo_timeslot(char *name)
{
	int first_timeslot;
	int res = 0;
	int chan;
	char *sptr;

	/* three possible cases here */
	/* mod (A)  , nonFXO(B)      */
	/* nonFXO(A), mod  (B)       */
	/* mod (A)  , mod  (B)       */

	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with (%s)\n",
			__FUNCTION__, name);

	if ((sptr = strstr(name, FXO_CHAN_HEADER))) {
		sptr += strlen(FXO_CHAN_HEADER);	
		chan = simple_strtol(sptr, NULL, 10);
	} else {
		printk(KERN_CRIT "%s() : BUG() FXO substring not found.\n",
			__FUNCTION__);
		return (res);
	}

	if ((moda == MODULE_TYPE_FXO) && (modb != MODULE_TYPE_FXO)) {
		first_timeslot = 2;
	} else if ((moda != MODULE_TYPE_FXO) && (modb == MODULE_TYPE_FXO)) {
		first_timeslot = 6;
	} else if ((moda == MODULE_TYPE_FXO) && (modb == MODULE_TYPE_FXO)) {
		first_timeslot = 2;
	} else {
		printk(KERN_ERR "%s() : BUG() No FXO module found.\n",
			__FUNCTION__);
		res = -1;
	}

	if (res == 0) {
		res = (first_timeslot + chan) << 1;
		dahdi_warp_log(DSP_ECAN_LOG, "%s() name (%s) => timeslot (%d)\n",
			__FUNCTION__, name, res);
	}

	return (res);
}

static int
echo_can_get_bri_timeslot(char *name)
{
	int modid, spanid, id;
	int first_timeslot;
	int res = 0;
	int parmCount;

	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with (%s)\n",
			__FUNCTION__, name);

	parmCount = sscanf(name, BRI_CHANNEL_FMT, &modid, &spanid, &id);
	if (parmCount == BRI_CHANNEL_PARAMCOUNT) {
		/*printk(KERN_DEBUG "%s() : %s - modid (%d) spanid(%d) id(%d)\n",
			__FUNCTION__, name, modid, spanid, id);*/
	} else {
		printk(KERN_ERR "%s() : BUG() Invalid BRI String [%s], parmCount = %d[expected %d]\n",
			__FUNCTION__, name, parmCount, BRI_CHANNEL_PARAMCOUNT);
		res = -1;
		return (res);
	}

	if (res == 0) {
		first_timeslot = (modid == 0) ? 4 : 12;
		res = first_timeslot + (spanid - 1) * 2 + (id - 1);
		dahdi_warp_log(DSP_ECAN_LOG, "%s() name (%s) => timeslot (%d)\n",
			__FUNCTION__, name, res);
	}

	return (res);
}

static int 
echo_can_get_timeslot(char *name)
{
	int res = -1;

	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with (%s)\n",
			__FUNCTION__, name);

	if (strstr(name, "FXS")) {
		res = echo_can_get_fxs_timeslot(name);
	} else if (strstr(name, "FXO")) {
		res = echo_can_get_fxo_timeslot(name);
	} else if (strstr(name, "B4")) {
		res = echo_can_get_bri_timeslot(name);
	} else {
		printk(KERN_CRIT "%s() : unknown module type (%s)\n",
			__FUNCTION__, name);
	}

	dahdi_warp_log(DSP_ECAN_LOG, "%s() name (%s) => timeslot (%d)\n",
			__FUNCTION__, name, res);

	return (res);
}


static int 
echo_can_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
		struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	struct warp_ec_pvt *pvt;
	size_t size = sizeof(**ec) + 2 * sizeof(int);
	int timeslot, dsp_channel, tap_length;
	int res = 0;
	int dsp_count = 1, dsp_id=0;

	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with channel name (%s)\n",
			__FUNCTION__, chan->name);

	if ((timeslot = echo_can_get_timeslot(chan->name)) >= 0) {
		res = 0;
		tap_length = ecp->tap_length;
		dahdi_warp_log(DSP_ECAN_LOG, "%s() Creating ECAN Timeslot = %d Tap Length = %d\n",
			__FUNCTION__, timeslot, tap_length);
	} else {
		printk(KERN_ERR "%s() : BUG() : Invalid timeslot %d for %s\n",
			__FUNCTION__, timeslot, chan->name);
		res = -EINVAL;
	}

	// get max dsp count
	dsp_count = warp_fpga_dsp_count();

	for ( dsp_id = 0; dsp_id < dsp_count; dsp_id++)
	{
		if ((res == 0) && (dahdi_warp_get_echocan_channel(dsp_id, timeslot, tap_length, &dsp_channel) == 0)) {
			res = 0; // Clean to valid state
			pvt = kzalloc(size, GFP_KERNEL);
			if (!pvt) {
				printk(KERN_ERR "%s() : Insufficient memory for a new ECAN channel on DSP %d\n",
						__FUNCTION__, dsp_id);
				res = -ENOMEM;
			}
			break;
		} else {
			printk(KERN_ERR "%s() : Insufficient resources for a new ECAN channel on DSP %d\n",
					__FUNCTION__, dsp_id);
			res = -ENOMEM;
		}
	}

	if (res == 0) {
		pvt->dahdi.ops = &warp_ecan_ops;
		pvt->dahdi.features = warp_ecan_features;
		pvt->dsp_channel = dsp_channel;
		pvt->timeslot = timeslot;
		pvt->dsp_id = dsp_id;
		dahdi_warp_log(DSP_ECAN_LOG, "%s() Name = %s Channel Number = %d Channel Position  = %d\n",
				__FUNCTION__, chan->name, chan->channo, chan->chanpos);
		*ec = &pvt->dahdi;
	} 	

	return (res);
}

static void
echo_can_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct warp_ec_pvt *pvt = dahdi_to_pvt(ec);
	int timeslot;
	int dsp_channel;

	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with channel name (%s)\n",
			__FUNCTION__, chan->name);

	timeslot = echo_can_get_timeslot(chan->name);
	dsp_channel = pvt->dsp_channel;

	dahdi_warp_log(DSP_ECAN_LOG, "%s() Name = %s Channel Number = %d Channel Position = %d Timeslot = %d Dsp Channel (%d) = %d\n",
			__FUNCTION__, chan->name, chan->channo, chan->chanpos, timeslot, pvt->dsp_id, dsp_channel);

	dahdi_warp_release_echocan_channel(pvt->dsp_id, dsp_channel);

	kfree(pvt);
}

static void 
echo_can_process(struct dahdi_echocan_state *ec, short *isig, 
		const short *iref, u32 size)
{
#if 0
	dahdi_warp_log(DSP_ECAN_LOG, "%s() called\n",
			__FUNCTION__);
#endif
}

static int
echo_can_traintap(struct dahdi_echocan_state *ec, int pos, short val)
{
	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with pos = %d val = %d\n",
			__FUNCTION__, pos, val);
	return 1;
}

static void 
echocan_NLP_toggle(struct dahdi_echocan_state *ec, unsigned int enable)
{
	dahdi_warp_log(DSP_ECAN_LOG, "%s() called with enable = %d\n",
			__FUNCTION__, enable);
}

static int __init mod_init(void)
{

	if ( warp_fpga_isactive() == 0 )
	{    
		printk(KERN_ERR "%s: FPGA is not active\n", __FUNCTION__);
		return -ENODEV;
	}  
	
	if ( warp_fpga_dsp_count() == 0 )
	{
		printk(KERN_ERR "%s: No DSP Found, failed to init echocan\n", __FUNCTION__);
		return -ENODEV;
	}


	if (dahdi_register_echocan_factory(&warp_ecan_factory)) {
		module_printk(KERN_ERR, "could not register with DAHDI core\n");
		return -EPERM;
	}

	get_module_types(&moda, &modb);

#if defined(DAHDI_2_5_0)
	module_printk(KERN_NOTICE, "Registered echo canceler '%s'.  Built %s %s\n", warp_ec_name, __DATE__, __TIME__);
#else
	module_printk(KERN_NOTICE, "Registered echo canceler '%s'.  Built %s %s\n", warp_ecan_factory.name, __DATE__, __TIME__);
#endif
	return 0;
}

static void __exit mod_exit(void)
{
	u32 dsp_count=0, dsp_id=0;
	dahdi_unregister_echocan_factory(&warp_ecan_factory);


	dsp_count = warp_fpga_dsp_count();
	for ( dsp_id = 0; dsp_id < dsp_count; dsp_id++)
	{
		dahdi_warp_release_all_echocan_channels(dsp_id);
	}
}

MODULE_DESCRIPTION("DAHDI 'WARP' Echo Canceler");
MODULE_AUTHOR("Utku Karaaslan");
MODULE_LICENSE("GPL v2");

module_init(mod_init);
module_exit(mod_exit);
