#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h> /*copy_from_user */
#include <linux/delay.h>
#include "n867a_ch7026_init_reg.h"
#include "vout_ch7026.h"

#define GPIO_NUM(group, num)  (group*NUM_GROUP + PIN_BASE + num)

unsigned char set_reg_addr[5][8] = {
	{0,     0,     0,     0,     0,     0,     0,     0    },
	{HTI_L, HTI_H, HAI_L, HAI_H, HO_L,  HO_H,  HW_L,  HW_H },	//1
	{VTI_L, VTI_H, VAI_L, VAI_H, VO_L,  VO_H,  VW_L,  VW_H },	//2
	{HTO_L, HTO_H, HAO_L, HAO_H, HOO_L, HOO_H, HWO_L, HWO_H},	//3
	{VTO_L, VTO_H, VAO_L, VAO_H, VOO_L, VOO_H, VWO_L, VWO_H}	//4
};

struct cdev	cdev;
struct class			*vout_ch7026_class;
struct cdev	cdev_adj;
struct class			*ch7026_class;

enum {POWER_DOWN=0, POWER_UP};
struct work_struct		vout_ch7026_wq;
static struct i2c_client *i2c_client = NULL;


int iic_reg_read(unsigned char reg_addr)
{
	return platform_iic_reg_read(i2c_client, reg_addr);
}

void iic_reg_write(unsigned char reg_addr, unsigned char data)
{
	platform_iic_reg_write(i2c_client, reg_addr, data);
}

#if 1
static void vout_ch7026_reset(void)
{
	vout_ch7026_platform_reset();
}
#endif

static int vout_ch7026_detected(void)
{
	unsigned char reg_val;
	int ret = -1;
	
	reg_val = iic_reg_read(0x00);
	if(0x54 == reg_val)
	{
		printk(KERN_INFO "OK: found ch7026 \n");
		ret = 0;
	}
	else
	{
		printk(KERN_ERR "Error: ch7026 does not exist. 0x%02x \n", reg_val);
		ret = -1;
	}
	
	return ret;
}

#if 1
static void vout_ch7026_init_reg(void)
{	
	unsigned char i;
	unsigned char reg_num;
#if (N867A_YPBPR_NTSC == 1)
	PCH7026REG p_reg = huabao_ntsc_data;
	reg_num = sizeof(huabao_ntsc_data)/sizeof(CH7026REG);
#elif (N867A_RGB == 1)
	PCH7026REG p_reg = RGB_init_data ;
	reg_num = sizeof(RGB_init_data)/sizeof(CH7026REG);
#endif

	printk(KERN_INFO "%s \n", __FUNCTION__);
	for(i=0; i<reg_num; i++)
	{
		iic_reg_write(p_reg->c_reg, p_reg->c_data);
		//printk("[wrt 0x%02x] = 0x%02x\n",p_reg->c_reg, iic_reg_read(p_reg->c_reg) );
		p_reg++;
	}	
}

static void vout_ch7026_powerup(int is_powerup)
{
	if(POWER_UP == is_powerup)
	{
		iic_reg_write(0x3D, 0x9E);
		iic_reg_write(0x3E, 0x21);
		iic_reg_write(0x04, 0x00); //power-up all DAC OUTPUT
	}
	else
	{
		iic_reg_write(0x04, 0xFF);	// power-down all channel
	}
}

static void vout_ch7026_init_SDRAM(void)
{
	int i;
	unsigned char dadi;
	
	iic_reg_write(0x06, 0x73); //start initialization when MEMINIT goes to low
	iic_reg_write(0x06, 0x71);
	
	dadi = iic_reg_read(0x7E);
	for(i=0; i<200; i++)
	{   //wait for initialization complete
		dadi = iic_reg_read(0x7E);
		if(dadi & 0x08)  //is SDRAM INIT COMPLETE
			break;
		mdelay(1);
	}
	
	iic_reg_write(0x06, 0x30);
}

static void vout_ch7026_wq_func(void)
{
	printk(KERN_INFO "--%s-- \n", __FUNCTION__);
	vout_ch7026_init_reg();
	vout_ch7026_powerup(POWER_UP);
	vout_ch7026_init_SDRAM();
}
#endif

int vout_ch7026_cdev_open(struct inode *inode,struct file *filp)
{
	//printk(KERN_INFO "++++ %s ++++\n", __FUNCTION__);
	return 0;
}

int vout_ch7026_cdev_release(struct inode *inode,struct file *filp)
{
	//printk(KERN_INFO "++++ %s ++++\n", __FUNCTION__);
	return 0;
}

static ssize_t vout_ch7026_cdev_read(struct file *filp,char __user *buf,size_t count, loff_t *ppos)
{
	unsigned char k_buf[64];
	int type;
	int reg[32];
	int val[32];
	int i   = 0;
	int num = 0; //number of reg

	memset( k_buf, 0, sizeof(k_buf) );
	if( copy_from_user(k_buf, buf, count) )
	{
		printk(KERN_ERR " %s copy buf data from user error. \n", __FUNCTION__);
		return -EFAULT;
	}

	type = k_buf[0];
	switch(type)
	{
		case 0:
			reg[0]   = k_buf[1];
			num = 1;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
			for(i=0; i<8; i++)
			{ //get reg 
				reg[i] = set_reg_addr[type][i];
			}
			num = 8;
			break;
		case 5:
			reg[0] = HP_L;
			reg[1] = HP_H;
			num = 2;
			break;
		case 6:
			reg[0] = VP_L;
			reg[1] = VP_H;
			num = 2;
			break;
		default:
			;
	}

	for(i=0; i<num; i++)
	{
		//read reg addr value
		val[i] = iic_reg_read(reg[i]);

		k_buf[2*i]   = reg[i];
		k_buf[2*i+1] = val[i];
	}

	if( copy_to_user(buf, k_buf, count) )
	{
		printk(KERN_ERR " %s copy data to user fail.\n", __FUNCTION__);
		return -EFAULT;
	}

	return 0;
}

static ssize_t vout_ch7026_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned char k_buf[32];
	unsigned char reg[32];
	unsigned char val[32];
	int type;
	int i;

	memset(k_buf, 0, sizeof(k_buf) );
	if( copy_from_user(k_buf, buf, count) )
	{
		printk(KERN_ERR " %s copy buf data from user error. \n", __FUNCTION__);
		return -EFAULT;
	}

	type = k_buf[0];
	switch(type)
	{
		case 0:
			reg[0] = k_buf[1];
			val[0] = k_buf[2];
			iic_reg_write(reg[0], val[0]);
			break;
		case 1:
		case 2:
		case 3:
		case 4:
			for(i=0; i<8; i++)
			{
				reg[i] = set_reg_addr[type][i];
				val[i] = k_buf[i+1];
				if(i%2 == 0)
				{ //write low reg, like HTO
					iic_reg_write(reg[i], val[i]);
				}
				else
				{
					//read high reg, like HTO
					if(i == 3 || i == 7)
					{ //e.g. like high reg of HTO and HAO are the same one, so is ignored
						continue;
					}
					else
					{ //i == 1, 5, step1 -- read reg value
						val[10+i] = iic_reg_read(reg[i]);
					}
				}
			}

			//step2 -- combine value for write
			if(type != 3)
			{
				val[21] = (val[11] & 0xC0) | (val[1] << 3) | val[3];
			}
			else
			{
				val[21] = (val[11] & 0x80) | (val[1] << 3) | val[3];
			}
			val[25] = (val[15] & 0xC0) | (val[7] << 3) | val[5];

			iic_reg_write(reg[1], val[21]);
			iic_reg_write(reg[5], val[25]);
			break;
		case 5:
			reg[0] = HP_L;
			reg[1] = HP_H;
			val[0] = k_buf[1];
			val[1] = k_buf[2];
			iic_reg_write(reg[0], val[0]);
			iic_reg_write(reg[1], val[1]);
			break;
		case 6:
			reg[0] = VP_L;
			reg[1] = VP_H;
			val[0] = k_buf[1];
			val[1] = k_buf[2];
			iic_reg_write(reg[0], val[0]);
			iic_reg_write(reg[1], val[1]);
			break;
		default:
			;
	}

	return 0;
}

#define CH7026_NORMAL_MODE	0
//#define CH7026_TEST_MODE	1
static long ch7026_cdev_ioctl(struct file * filep, unsigned int cmd, unsigned long arg)
{
	unsigned char value;
	static unsigned char changeflag = 0;
	static unsigned char reg[2] = {0};
	static unsigned int  temp_val;

	//printk("CH7026_TEST_MODE:\n");
#if 1
	if(CH7026_NORMAL_MODE != cmd)//1~9:Test mode
	{
		reg[0] = 0x04;
		value = cmd - 1;
		temp_val = iic_reg_read(reg[0]);
		
		iic_reg_write(0x03, 0x01);
		iic_reg_write(reg[0], value|0x20);
		changeflag = 1;
	}
	else
	{//Recover to the last normal value
		if(changeflag == 1)
		{
			iic_reg_write(reg[0], temp_val);
			iic_reg_write(0x03, 0x00);
		}
		changeflag = 0;
	}
#else
	value = (cmd&0x20)>>5;	//BIT5	-->1:test mode 0:normal mode
	if(CH7026_TEST_MODE != value)
	{
		reg[0] = 0x04;
		temp_val = iic_reg_read(reg[0]);
		
		iic_reg_write(reg[0], cmd);
		changeflag = 1;
	}
	else
	{
		if(changeflag == 1)
		{
			iic_reg_write(reg[0], temp_val);
		}
		changeflag = 0;
	}
#endif
	if( copy_to_user((int *)arg, &changeflag, 1) )
	{
		return -EFAULT;
	}
	return 0;
}

struct file_operations vout_ch7026_cdev_fops =
{
	.owner   = THIS_MODULE,
	.read    = vout_ch7026_cdev_read,
	.write   = vout_ch7026_cdev_write,
	.unlocked_ioctl	 = ch7026_cdev_ioctl,
	.open    = vout_ch7026_cdev_open,
	.release = vout_ch7026_cdev_release,
};

int vout_ch7026_cdev_init(void)
{
	int result;
	int err;

	dev_t devno = MKDEV(CH7026_MAJOR,0);
	if(CH7026_MAJOR)
	{
		result = register_chrdev_region(devno, 1, "ch7026_ch");
	}
	else
	{
		result = alloc_chrdev_region(&devno, 0, 1, "ch7026_ch");
		//CH7026_MAJOR = MAJOR(devno);
	}
	if(result < 0)
	{
		printk (KERN_WARNING "hello: can't get major number %d/n", CH7026_MAJOR);
		return result;
	}


	cdev_init(&cdev, &vout_ch7026_cdev_fops);
	cdev.owner = THIS_MODULE;
	cdev.ops   = &vout_ch7026_cdev_fops;
	err	       = cdev_add(&cdev, devno, 1);
	if(err)
	{
		printk(KERN_NOTICE "Error %d adding ch7026 \n", err);
	}

	vout_ch7026_class = class_create(THIS_MODULE, "ch7026_class");
	if(IS_ERR(vout_ch7026_class))
	{
		printk("Err: failed in creating class./n");
		return -1; 
	} 
	device_create( vout_ch7026_class, NULL, devno, NULL,"ch7026_debug");

	return 0;
}


/* *******************************************
 * 
 * Adjust CH7026 for APP use JNI
 *    type
 *    0 --- Contrast   (30H) 
 *    1 --- Saturation (2FH) 
 *    2 --- Brightness (31H)
 *    3 --- Hue        (2EH)
 *    4 --- Vertical Position   (33H,34H)
 *    5 --- Horizontal Position (35H,36H)
 *    6 --- Stretch && Compress in Horizontal (1BH,1CH)
 *    7 --- Stretch && Compress in Vertical (21H,22H)
 * 
 ****************************************** */

int ch7026_cdev_adj_open(struct inode *inode,struct file *filp)
{
	//printk(KERN_INFO "++++ %s ++++\n", __FUNCTION__);
	return 0;
}

int ch7026_cdev_adj_release(struct inode *inode,struct file *filp)
{
	//printk(KERN_INFO "++++ %s ++++\n", __FUNCTION__);
	return 0;
}

static ssize_t ch7026_cdev_adj_read(struct file *filp,char __user *buf,size_t count, loff_t *ppos)
{
	unsigned char cmd_buf[32];
	unsigned char type;
	unsigned char reg[16] = {0};
	unsigned char val[16];
	unsigned char temp_val;
	int i;

	memset(cmd_buf, 0, sizeof(cmd_buf) );
	if( copy_from_user(cmd_buf, buf, count) )
	{
		return -EFAULT;
	}

	type = cmd_buf[0];

	switch(type)
	{
		case 0:		// 0x00 ~ 0x7F
			reg[0] = CONTRAST;
			break;
		case 1:		// 0x00 ~ 0x7F
			reg[0] = SATURATION;
			break;
		case 2:		// 0x00 ~ 0xFF
			reg[0] = BRIGHTNESS;
			break;
		case 3:		// 0x00 ~ 0x7F
			reg[0] = HUE;
			break;
		case 4:		// VP
			reg[0] = VP_H;
			reg[1] = VP_L;
			break;
		case 5:		// HP
			reg[0] = HP_H;
			reg[1] = HP_L;
			break;
		case 6:		// Stretch Width HAO
			reg[0] = HAO_H;
			reg[1] = HAO_L;
			reg[2] = HTO_H;
			reg[3] = HTO_L;
			reg[4] = HOO_H;
			reg[5] = HOO_L;
			reg[6] = HWO_H;
			reg[7] = HWO_L;
			break;
		case 7:		// Stretch High VAO
			reg[0] = VAO_H;
			reg[1] = VAO_L;
			reg[2] = VTO_H;
			reg[3] = VTO_L;
			reg[4] = VOO_H;
			reg[5] = VOO_L;
			reg[6] = VWO_H;
			reg[7] = VWO_L;
			break;
		/* Rotaton 0 90 180 270 */
		case 8:
			reg[0] = ROTATION;
			break;
		/* Flip in H or V */
		case 9:
			reg[0] = FLIP;
			break;
		/* Text enhancement */
		case 10:
			reg[0] = TEXT_EH;
			break;
		default:
			;
	}

	if(type < 6)
	{
		val[0] = iic_reg_read(reg[0]);
		val[1] = iic_reg_read(reg[1]);
		//printk("read 0x%02x  = 0x%02x \n", reg[0], val[0] );

		cmd_buf[0] = reg[0];
		cmd_buf[1] = val[0];
		cmd_buf[2] = reg[1];
		cmd_buf[3] = val[1];
	}
	else if(type < 8)		// ==6,7
	{
		for(i=0; i<8; i++)
		{
			val[i] = iic_reg_read(reg[i]);
		}

		val[0] = val[0] & 0x07;
		val[2] = val[2] >> 3;
		val[4] = val[4] & 0x07;
		val[6] = val[6] >> 3;

		for(i=0; i<8; i++)
		{
			cmd_buf[i] = val[i];
		}
	}
	else if(type < 11)
	{
		temp_val = iic_reg_read(reg[0]);

		switch(type)
		{
			case 8:
				val[0] = temp_val & 0x03;
				break;
			case 9:
				val[0] = (temp_val & 0x0C) >> 2;
				break;
			case 10:
				val[0] = temp_val & 0x07;
				break;
			default:
				;
		}

		cmd_buf[0] = val[0];
	}

	if( copy_to_user(buf, cmd_buf, 30) )
	{
		return -EFAULT;
	}

	return 0;
}

static ssize_t ch7026_cdev_adj_write(struct file *filp,const char __user *buf,size_t count, loff_t *ppos)
{
	unsigned char cmd_buf[32];
	unsigned int  type;
	unsigned char reg[2] = {0};
	unsigned int  val[3];	//0--low, 1--high, 2--whole val;
	unsigned int  temp_val = 0;

	memset(cmd_buf, 0, sizeof(cmd_buf) );
	if( copy_from_user(cmd_buf, buf, count) )
	{
		return -EFAULT;
	}

	type   = cmd_buf[0];
	val[0] = cmd_buf[1];	//low
	val[1] = cmd_buf[2];	//high
	val[2] = val[1]<<8 | val[0];	//whole value

	//printk("type:0x%02x, val:%d\n", type, val[2]);

	switch(type)
	{
		case 0x00:		// 0x00 ~ 0x7F
			reg[0] = CONTRAST;
			break;
		case 0x01:		// 0x00 ~ 0x7F
			reg[0] = SATURATION;
			break;
		case 0x02:		// 0x00 ~ 0xFF
			reg[0] = BRIGHTNESS;
			break;
		case 0x03:		// 0x00 ~ 0x7F
			reg[0] = HUE;
			break;
		case 0x40:		// UP, 0x00 ~ 0xFFF
			reg[0] = VP_L;
			reg[1] = VP_H;
			val[1] = val[1] | 0x08;
			break;
		case 0x41:		// DOWN, 0x00 ~ 0xFFF
			reg[0] = VP_L;
			reg[1] = VP_H;
			val[2] = 0x800 - val[2];
			val[1] = val[2] >> 8;
			val[0] = val[2] & 0xFF;
			break;
		case 0x50:		// RIGHT, 0x00 ~ 0xFFF
			reg[0] = HP_L;
			reg[1] = HP_H;
			val[1] = val[1] | 0x08;
			break;
		case 0x51:		// LEFT, 0x00 ~ 0xFFF
			reg[0] = HP_L;
			reg[1] = HP_H;
			val[2] = 0x800 - val[2];
			val[1] = val[2] >> 8;
			val[0] = val[2] & 0xFF;
			break;
		case 0x06:
			reg[0] = HAO_L;
			reg[1] = HAO_H;
			val[2] = iic_reg_read(reg[1]);
			val[2] = val[2] & 0xF8;
			val[1] = val[2] | val[1];
			break;
		case 0x07:
			reg[0] = VAO_L;
			reg[1] = VAO_H;
			val[2] = iic_reg_read(reg[1]);
			val[2] = val[2] & 0xF8;
			val[1] = val[2] | val[1];
			break;
		/* Rotation 0 90 180 270 */
		case 0x08:
			reg[0]   = ROTATION;
			temp_val = iic_reg_read(reg[0]);
			val[0]   = (temp_val & 0xFC) | val[0];
			break;
		/* HFLIP && VFLIP */
		case 0x09:
			reg[0] = FLIP;
			temp_val = iic_reg_read(reg[0]);
			val[0]   = (temp_val & 0xF3) | (val[0] << 2);
			break;
		/* Text Enhancement */
		case 0x0a:
			reg[0] = TEXT_EH;
			temp_val = iic_reg_read(reg[0]);
			val[0]   = (temp_val & 0xF8) | val[0];
			break;
		default:
			;
	}

	if(type < 4 || (type >= 8 && type <= 10) )
	{
		iic_reg_write(reg[0], val[0]);
	}
	else	//40, 41, 50, 51, 6, 7
	{
		iic_reg_write(reg[1], val[1]);
		iic_reg_write(reg[0], val[0]);
		//printk("reg[1]: 0x%02x\n", val[1]);
		//printk("reg[0]: 0x%02x\n", val[0]);
	}

	return 0;
}

struct file_operations ch7026_cdev_adj_fops =
{
	.owner   = THIS_MODULE,
	.read    = ch7026_cdev_adj_read,
	.write   = ch7026_cdev_adj_write,
	.open    = ch7026_cdev_adj_open,
	.release = ch7026_cdev_adj_release,
};

int ch7026_cdev_adj_init(void)
{
	int result;
	int err;

	dev_t devno = MKDEV(CH7026_ADJ_MAJOR,0);
	if(CH7026_ADJ_MAJOR)
	{
		result = register_chrdev_region(devno, 1, "ch7026_adj_ch");
	}
	else
	{
		result = alloc_chrdev_region(&devno, 0, 1, "ch7026_adj_ch");
		//CH7026_ADJ_MAJOR = MAJOR(devno);
	}
	if(result < 0)
	{
		printk (KERN_WARNING "hello: can't get major number %d/n", CH7026_ADJ_MAJOR);
		return result;
	}


	cdev_init(&cdev_adj, &ch7026_cdev_adj_fops);
	cdev_adj.owner = THIS_MODULE;
	cdev_adj.ops   = &ch7026_cdev_adj_fops;
	err	           = cdev_add(&cdev_adj, devno, 1);
	if(err)
	{
		printk(KERN_NOTICE "Error %d adding ch7026 \n", err);
	}

	ch7026_class = class_create(THIS_MODULE, "ch7026_adj_class");
	if(IS_ERR(ch7026_class))
	{
		printk("Err: failed in creating class./n");
		return -1; 
	} 
	device_create( ch7026_class, NULL, devno, NULL,"ch7026_setvideoeffect");

	return 0;
}


ssize_t gpio_show(struct device *dev,struct device_attribute *attr, char *buf)
{    
	printk("\t echo \"214 1 1\">gpio       --- set GPIO_NUM(2, 14) output 1\n");
	printk("\t echo \"027 1 0\">gpio       --- set GPIO_NUM(0, 27) output 0\n");
	printk("\t echo \"028 0\">gpio         --- get GPIO_NUM(0, 28) pin value\n");
	printk("\n");
	
	return sprintf(buf, "ch7026 gpio_show");
}

ssize_t gpio_store(struct device *dev, struct device_attribute *attr, const char * buf, size_t count)
{
	unsigned int reg ;
	unsigned int in_out;
	unsigned int val ;
	unsigned int group, num ;


	//vout_ch7026_reset();

	//vout_ch7026_init_reg();

	sscanf(buf, "%d %d %d", &reg, &in_out, &val);
	
	group = reg/100;
	num   = reg%100;
	printk("group:%d\n", group);
	printk("num ::%d\n", num);
	
	if((group < 0 || group > 6 || group == 5) || (num < 0 || num > 31))
	{
		printk(KERN_ERR"error gpio index");
	}
	//gpio_pull(GPIO_NUM(group, num), 1);
	gpio_pull_updown(GPIO_NUM(group, num), 1);
	//gpio_pull_high(GPIO_NUM(group, num), 1);

	if(1 == in_out)  	//OUTPUT
	{
		gpio_direction_output(GPIO_NUM(group, num), 1);
		if(0 != val)	//gpio set output 1
		{
			gpio_set_value(GPIO_NUM(group, num), 1);	
		}
		else 	//gpio set output 0
		{
			gpio_set_value(GPIO_NUM(group, num), 0);	
		}
	}
	else 	//INPUT
	{
		gpio_direction_input(GPIO_NUM(group, num));
	}

	return strnlen(buf, PAGE_SIZE);
}

	
static DEVICE_ATTR(gpio, S_IRUGO|S_IWUSR, gpio_show, gpio_store);

static struct attribute *vout_ch7026_attrs[] = {
	&dev_attr_gpio.attr, 
	NULL 
};

static struct attribute_group vout_ch7026_group = {
	.name = NULL,
	.attrs = vout_ch7026_attrs,
};

static int vout_ch7026_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
	int i;
	int retval;
	unsigned char detect_ch7026;
	printk(KERN_INFO "-----%s------ \n", __FUNCTION__);
	
	i2c_client = client;
	
	vout_ch7026_reset();
	
	for(i=0; i<3; i++)
	{
		detect_ch7026 = vout_ch7026_detected();
		if(detect_ch7026 == 0 )
			break;
	}

	if(i==3 && detect_ch7026!=0)
	{
	//	return 0;
	}

	//INIT_WORK(&vout_ch7026_wq, vout_ch7026_wq_func);
	//schedule_work(&vout_ch7026_wq);
	
	// add sys debug
	retval = sysfs_create_group(&client->dev.kobj, &vout_ch7026_group);
	if (retval) {
		printk(KERN_ERR "Can't create sysfs attrs for vty-server@%X\n", retval);
		return retval;
	}

	//vout_ch7026_wq_func();
	vout_ch7026_init_reg();

	return 0;
}

static int vout_ch7026_remove(struct i2c_client *client)
{
	//printk(KERN_INFO "-----%s--d_wq2---- \n", __FUNCTION__);
	
	if( !work_pending(&vout_ch7026_wq) )
	{
		cancel_work_sync(&vout_ch7026_wq);
	}
	
	i2c_client = NULL;
	
	sysfs_remove_group(&client->dev.kobj, &vout_ch7026_group);
	
	return 0;
}

static const struct i2c_device_id	vout_ch7026_id[] =
{
	{ "vout_ch7026", 0 },
	{  },
};

static struct i2c_driver	vout_ch7026_dirver = 
{
	.driver = 
	{
		.name = "vout_ch7026",
	},
	.probe = vout_ch7026_probe,
	.remove = vout_ch7026_remove,
	.id_table = vout_ch7026_id,
};

static int __init vout_ch7026_module_init(void)
{
	//printk(KERN_INFO "-----%s------ \n", __FUNCTION__);
	vout_ch7026_cdev_init();
	// Adjust for Saturation(2FH) Contrast(30H) Brightness(31H)
	ch7026_cdev_adj_init();
	vout_ch7026_platform_init();
	return i2c_add_driver(&vout_ch7026_dirver);
}

static void __exit vout_ch7026_module_exit(void)
{
	//printk(KERN_INFO "-----%s------ \n", __FUNCTION__);
	device_destroy(vout_ch7026_class, MKDEV(CH7026_MAJOR,0));
	class_destroy(vout_ch7026_class);
	cdev_del(&cdev);
	unregister_chrdev_region(MKDEV(CH7026_MAJOR,0),1);

	device_destroy(ch7026_class, MKDEV(CH7026_ADJ_MAJOR,0));
	class_destroy(ch7026_class);
	cdev_del(&cdev_adj);
	unregister_chrdev_region(MKDEV(CH7026_ADJ_MAJOR,0),1);

	vout_ch7026_platform_exit();
	i2c_del_driver(&vout_ch7026_dirver);
}

module_init(vout_ch7026_module_init);
module_exit(vout_ch7026_module_exit);

MODULE_DESCRIPTION("NWD CH7026 CVBS-output display");
MODULE_AUTHOR("Alex <os-huangfujun@nowada.com>");
MODULE_LICENSE("GPL v2");
