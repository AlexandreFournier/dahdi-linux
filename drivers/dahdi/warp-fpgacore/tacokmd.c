#include <pika/pikacore.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <pika/driver.h>
#include <pika/pksystemver.h>

/* fpga revision for new module type read */
#define FPGA_REVISION_2PART_READ	6086

//static void taco_print_rev(PDEVICE_EXTENSION pdx);

int taco_proc_init(PDEVICE_EXTENSION pdx);
int taco_proc_remove(void);

/* -------------------------------------------------------------- */

static char *taco_mod_type_to_str(unsigned type)
{
	switch (type) {
	case 0xf:
		return "empty";
	case 1:
		return "FXO";
	case 2:
		return "FXS";
	case 3:
		return "BRI";
	case 4:
		return "GSM";
	case 5:
		return "GSM";
	case 6:
		return "UMTS";
	case 7:
		return "PRI";
	case 8:
		return "LTE";
	case 0x7f:
		return "empty";
	default:
		return "???";
	}
}

static int taco_proc_show(struct seq_file *m, void *v)
{
	PDEVICE_EXTENSION pdx = m->private;
	unsigned rev = BAR0_READ(pdx, BAR0_FPGA_REV);
	unsigned fpga_rev = rev & 0xffff;
	
	if (fpga_rev >= FPGA_REVISION_2PART_READ) {
		return seq_printf(m, "FPGA %08X  %s/%s\n", rev, 
		               taco_mod_type_to_str(((rev >> 16) & 0xf) | (((rev >> 24) & 0x7) << 4)),
		               taco_mod_type_to_str(((rev >> 20) & 0xf) | (((rev >> 27) & 0x7) << 4)));
	} else { 
		return seq_printf(m, "FPGA %08X  %s/%s\n", rev, 
		               taco_mod_type_to_str((rev >> 16) & 0xf),
		               taco_mod_type_to_str((rev >> 20) & 0xf));
	}
}

static int taco_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, taco_proc_show, PDE_DATA(file_inode(file)));
}

static const struct file_operations taco_proc_fops = {
	.owner   	= THIS_MODULE,
	.open		= taco_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int taco_dspnum_proc_show(struct seq_file *m, void *v)
{
	return seq_printf(m, "%d\n", warp_fpga_dsp_count());
}

static int taco_dspnum_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, taco_dspnum_proc_show, PDE_DATA(file_inode(file)));
}

static const struct file_operations taco_dspnum_proc_fops = {
	.owner   	= THIS_MODULE,
	.open		= taco_dspnum_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int taco_proc_init(PDEVICE_EXTENSION pdx)
{
	if (!proc_create_data("driver/taco", 0, NULL, &taco_proc_fops, pdx))
		return -ENOMEM;
	if (!proc_create_data("driver/dsp_count", 0, NULL, &taco_dspnum_proc_fops, pdx))
		return -ENOMEM;

	return 0;
}

int taco_proc_remove(void)
{
	remove_proc_entry("driver/taco", NULL);
	remove_proc_entry("driver/dsp_count", NULL);

	return 0;
}

/*
 * Force kernel coding standard on PIKA code
 * Local Variables:
 * tab-width: 8
 * c-basic-offset: 8
 * indent-tabs-mode: t
 * End:
 */
