#include <linux/proc_fs.h>
#include <plat/board.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <mach/gpio.h> 
#include "vout_ch7026.h"


//#define GPIO_CH7026_RESET RK30_PIN4_PC1 //yinglong 
#define GPIO_CH7026_RESET RK30_PIN4_PB7//yinglong 
//static unsigned int PWM_BASE = 0;
struct proc_dir_entry	*proc_vout_ch7026 = NULL;

void platform_iic_reg_write(struct i2c_client *client, unsigned char reg_addr, unsigned char val)
{	
	struct i2c_msg msg;
	unsigned char buf[2];
	struct i2c_adapter *adapter = client->adapter;

	if (NULL==adapter){
		printk("%s: adapter==NULL\n", __FUNCTION__);
		return ;
	}

	buf[0] = reg_addr;
	buf[1] = val;

	msg.addr  = client->addr;
	msg.flags = 0x00;
	msg.len   = 2;
	msg.buf   = buf;
    	msg.scl_rate = 100 * 1000;    // for Rockchip
    	msg.udelay = 5;

	i2c_transfer(adapter, &msg, 1);
}

int platform_iic_reg_read(struct i2c_client *client, unsigned char reg_addr)
{		
	struct i2c_msg msg[2];
	unsigned char buf[2];
	unsigned int ret = -1;
	struct i2c_adapter *adapter = client->adapter;

	if (NULL==adapter){
		printk("%s: adapter==NULL\n", __FUNCTION__);
		return -1;
	}

	buf[0] = reg_addr;
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 1;
	msg[0].buf   = &buf[0];
    	msg[0].scl_rate = 100 * 1000;    // for Rockchip
    	msg[0].udelay = 5;
	//i2c_transfer(adapter, &msg[0], 1);

	msg[1].addr  = client->addr;
	msg[1].flags = 0x01;
	msg[1].len   = 1;
	msg[1].buf   = &buf[1];
    	msg[0].scl_rate = 100 * 1000;    // for Rockchip
    	msg[0].udelay = 5;
	ret = i2c_transfer(adapter, msg, 2);
	//ret = i2c_transfer(adapter, &msg[1], 1);

	return buf[1];
}

void vout_ch7026_stop(void)
{
	gpio_set_value(GPIO_CH7026_RESET, 0);
	mdelay(10);
}

void vout_ch7026_platform_reset(void)
{
	gpio_request(GPIO_CH7026_RESET, "CH7026_RST");
	gpio_direction_output(GPIO_CH7026_RESET, 0);
	mdelay(1000);
	gpio_set_value(GPIO_CH7026_RESET, 1);
}


static int vout_ch7026_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char cmd_buf[64];
	char *a1 = NULL;
	char *a2 = NULL;
	char *a3 = NULL;
	//int  addr;
	int  reg;
	int  val;
	int  data1;

	memset(cmd_buf, 0, sizeof(cmd_buf) );
	if( copy_from_user(cmd_buf, buff, len) )
	{
		return -EFAULT;
	}

	if( !strncmp(cmd_buf, "pwm", 3) )
	{
		printk("%s, %s for a6, no use in rk", __func__, __FILE__);
	/*
		a3 = cmd_buf + 9;
		a2 = cmd_buf + 4;

		if( !strncmp(a2, "0x", 2) )
		{
			a1 = cmd_buf;
			*(a1+3) = '\0';
			*(a2+4) = '\0';

			if( !strncmp(a3, "0x", 2) )
			{
				reg  = simple_strtoul(a2, NULL, 0);
				val  = simple_strtoul(a3, NULL, 0);
				printk("reg  = 0x%02x\n", reg);
				printk("val  = 0x%08x\n", val);

				//writel(val, PWM_BASE + reg);
				//data1 = readl(PWM_BASE + reg);
				printk("[pwm 0x%02x] = 0x%08x\n",reg, data1);
			}
			else
			{
				reg  = simple_strtoul(a2, NULL, 0);
				printk("reg  = 0x%02x\n", reg);

				//data1 = readl(PWM_BASE + reg);
				printk("[pwm 0x%02x] = 0x%08x\n",reg, data1);
			}
		}

		return len;
		*/
	}
	else if( !strncmp(cmd_buf, "wrt", 3) )
	{
		a3 = cmd_buf + 9;
		a2 = cmd_buf + 4;

		if( !strncmp(a2, "0x", 2) )
		{
			a1 = cmd_buf;
			*(a1+3) = '\0';
			*(a2+4) = '\0';

			if( !strncmp(a3, "0x", 2) )
			{
				reg  = simple_strtoul(a2, NULL, 0);
				val  = simple_strtoul(a3, NULL, 0);
				printk("reg  = 0x%02x\n", reg);
				printk("val  = 0x%02x\n", val);

				iic_reg_write(reg, val);

				data1 = iic_reg_read(reg);
				printk("[wrt 0x%02x] = 0x%02x\n",reg, data1);				
			}
			else
			{
				reg  = simple_strtoul(a2, NULL, 0);
				printk("reg  = 0x%02x\n", reg);

				data1 = iic_reg_read(reg);
				printk("[wrt 0x%02x] = 0x%02x\n",reg, data1);
			}
		}
		else if( !strncmp(a2, "readall", 2) )
		{
			printk("\n---------- CH7026 Reg 0x00~0x7F ---------\n");
			for(reg=0; reg<=0x7F; reg++)
			{
				data1 = iic_reg_read(reg);
				if(reg%2 != 0)
				{
					printk("[  0x%02x   0x%02x] \n",reg, data1);
					continue;
				}
				printk("[0x%02x   0x%02x] \n",reg, data1);
			}
			printk("\n");

			iic_reg_write(0x1C, 0x9F);
			data1 = iic_reg_read(0x1C);
			printk("[wrt 0x%02x] = 0x%02x\n",0x1C, data1);

			printk("\n");
		}

		return len;
	}
	else if( !strncmp(cmd_buf, "help", 4) )
	{
		printk("\t echo \"pwm 0x08\">/proc/ch7026                  --- read PWM register\n");
		printk("\t echo \"pwm 0x08 0x12345678\">/proc/ch7026       --- write PWM register\n");
		printk("\t echo \"wrt 0x06\">/proc/ch7026                  --- read CH7026 reg 0x06\n");
		printk("\t echo \"wrt 0x06 0xaa\">/proc/ch7026             --- write 0xaa to CH7026 0x06 \n");
		printk("\t echo \"wrt readall\">/proc/ch7026               --- readall CH7026 reg \n");

		return len;
	}

	return len;
}

#if 0 
void vout_ch7026_clock_enable(void)
{
	// X_CKO_0, PWM4
	// Freq = 13MHZ
	PWM_BASE = (unsigned int)ioremap(SIRFSOC_PWMMOD_PA_BASE, SIRFSOC_PWMMOD_SIZE-1);	
	writel( readl(GPIO_BASE + GPIO_PAD_EN(2)) & ~(1<<14), GPIO_BASE + GPIO_PAD_EN(2) );	// Disable GPIO

	writel( readl(PWM_BASE + 0x04) & ~(0x1<<4),  PWM_BASE + 0x04);	// Disable PWM4
	mdelay(5);
	writel( readl(PWM_BASE + 0x04) & ~(0x1<<11),  PWM_BASE + 0x04);	// Disable PWM4 ratio
	mdelay(5);
	writel( readl(PWM_BASE + 0x0C) & ~(0x1<<4), PWM_BASE + 0x0C);	// Disable PWM4 Post-clock
	mdelay(5);
	writel( readl(PWM_BASE + 0x08) & ~(0x1<<4), PWM_BASE + 0x08);	// Disable PWM4 Pre-clock
	mdelay(5);
	writel( readl(GPIO_BASE + GPIO_PAD_EN(2)) & ~(1<<14), GPIO_BASE + GPIO_PAD_EN(2) );	// Disable GPIO
	writel( readl(PWM_BASE + 0x00) & ~(0x7<<12), PWM_BASE + 0x00);	// Pre-clock source select: xin-clock
	mdelay(5);
	writel( readl(PWM_BASE + 0x08) | (0x1<<4), PWM_BASE + 0x08);	// Enable PWM4 Pre-clock
	writel( readl(PWM_BASE + 0x00) & ~(0x01<<25), PWM_BASE + 0x00);	// Clear bypass selection
	writel( 0x00, PWM_BASE + 0x30);	// Number of High pulse
	writel( 0x00, PWM_BASE + 0x34);	// Number of Low pulse
	writel( readl(PWM_BASE + 0x04) | (0x1<<11),  PWM_BASE + 0x04);	// Enable PWM4 ratio
	writel( readl(PWM_BASE + 0x0C) | (0x1<<4), PWM_BASE + 0x0C);	// Enable PWM4 Post-clock
	writel( readl(PWM_BASE + 0x04) | (0x1<<4),  PWM_BASE + 0x04);	// Enable PWM4

}
#endif

int vout_ch7026_platform_init(void)
{
	//printk(KERN_INFO "--28---%s------ \n", __FUNCTION__);
	// X_CKO_0, PWM4, 13MHZ
	//vout_ch7026_clock_enable();
	proc_vout_ch7026 = create_proc_entry("ch7026", 0666, NULL);
	if( NULL == proc_vout_ch7026 )
	{
		printk(KERN_INFO "%s:  Cound't create proc Entry \n", __FUNCTION__ );
		return -ENOMEM;
	}
	proc_vout_ch7026->write_proc = vout_ch7026_write;

	return 0;
}

void vout_ch7026_platform_exit(void)
{
	//iounmap( (void *)PWM_BASE );
	remove_proc_entry("ch7026", NULL);
}
