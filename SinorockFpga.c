/***************************************************************************
 *
 * Copyright (C) 2010-2011 SINOROCK
 * auther:maoyq
 * date:2010.12.15
 *
 * GPMC cs3----FPGA  A[1-10]  D[0-15]
 ***************************************************************************/
#include "SinorockFpga.h"
#include <linux/clk.h>
#include <plat/clock.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/ioctl.h>
#include <plat/gpmc.h>

//ioctl cmd define 
#define SINOROCK_FPGA_MAGIC  'P'

#define IOCTRL_FPGA_START_SAMPLE_EN       _IOW(SINOROCK_FPGA_MAGIC,1,int)//开始采集
#define IOCTRL_FPGA_STOP_SAMPLE_EN        _IOW(SINOROCK_FPGA_MAGIC,2,int) //结束采集
#define IOCTRL_FPGA_SET_SAMPLE_NUMBER_EN  _IOW(SINOROCK_FPGA_MAGIC,3,int)//设置采样点数
#define IOCTRL_FPGA_SET_SAMPLE_RATE_EN    _IOW(SINOROCK_FPGA_MAGIC,4,int) //设置采样间隔
#define IOCTRL_FPGA_SET_DELAY_POINT_EN    _IOW(SINOROCK_FPGA_MAGIC,5,int)//设置延时点数
#define IOCTRL_FPGA_SET_INIT_ADDR_FOR_READ_DATA_EN 	_IOW(SINOROCK_FPGA_MAGIC,6,int)//设置读取数据初地址
#define IOCTRL_FPGA_START_BATTERY_SAMPLE_EN     	_IOW(SINOROCK_FPGA_MAGIC,7,int)//开始电量转换
#define IOCTRL_FPGA_TRIG_ENABLE_OR_DISABLE_EN 		_IOW(SINOROCK_FPGA_MAGIC,8,int) //触发使能有效与否，发1表示触发有效，发0表示触发无效
#define IOCTRL_FPGA_TRIGGER_POS_EN 	_IOW(SINOROCK_FPGA_MAGIC,9,int) //设置触发最大值
#define IOCTRL_FPGA_TRIGGER_NEG_EN 	_IOW(SINOROCK_FPGA_MAGIC,10,int) //设置触发最大值
#define IOCTRL_FPGA_SET_CF_WIDE_EN 	_IOW(SINOROCK_FPGA_MAGIC,11,int)//设置发射脉宽
#define IOCTRL_FPGA_SET_CF_LOW_OR_HIGH_EN 	_IOW(SINOROCK_FPGA_MAGIC,12,int) //设置发射高低电平,1表示高，0表示低
#define IOCTRL_FPGA_SET_RELAY_EN 	_IOW(SINOROCK_FPGA_MAGIC,13,int)//设置继电器工作情况
#define IOCTRL_FPGA_HIGH_CLEAR_EN 	_IOW(SINOROCK_FPGA_MAGIC,14,int) //高度清零
#define IOCTRL_FPGA_A_AMP_CH1_EN 	_IOW(SINOROCK_FPGA_MAGIC,15,int)//设置加速度通道1的放大倍数
#define IOCTRL_FPGA_A_AMP_CH2_EN 	_IOW(SINOROCK_FPGA_MAGIC,16,int)//设置加速度通道2的放大倍数
#define IOCTRL_FPGA_A_AMP_CH3_EN 	_IOW(SINOROCK_FPGA_MAGIC,17,int)//设置加速度通道3的放大倍数
#define IOCTRL_FPGA_A_AMP_CH4_EN 	_IOW(SINOROCK_FPGA_MAGIC,18,int)//设置加速度通道4的放大倍数
#define IOCTRL_FPGA_S_AMP_CH1_EN 	_IOW(SINOROCK_FPGA_MAGIC,19,int)//设置应变通道1的放大倍数
#define IOCTRL_FPGA_S_AMP_CH2_EN 	_IOW(SINOROCK_FPGA_MAGIC,20,int)//设置应变通道2的放大倍数
#define IOCTRL_FPGA_S_AMP_CH3_EN 	_IOW(SINOROCK_FPGA_MAGIC,21,int)//设置应变通道3的放大倍数
#define IOCTRL_FPGA_S_AMP_CH4_EN 	_IOW(SINOROCK_FPGA_MAGIC,22,int)//设置应变通道4的放大倍数
#define IOCTRL_FPGA_PWM_A_FILTER_CH1_EN 	_IOW(SINOROCK_FPGA_MAGIC,23,int) //设置加速度CH1的滤波参数
#define IOCTRL_FPGA_PWM_A_FILTER_CH2_EN 	_IOW(SINOROCK_FPGA_MAGIC,24,int) //设置加速度CH2的滤波参数
#define IOCTRL_FPGA_PWM_A_FILTER_CH3_EN 	_IOW(SINOROCK_FPGA_MAGIC,25,int) //设置加速度CH3的滤波参数
#define IOCTRL_FPGA_PWM_A_FILTER_CH4_EN 	_IOW(SINOROCK_FPGA_MAGIC,26,int) //设置加速度CH4的滤波参数
#define IOCTRL_FPGA_PWM_S_FILTER_CH1_EN 	_IOW(SINOROCK_FPGA_MAGIC,27,int) //设置应变CH1的滤波参数
#define IOCTRL_FPGA_PWM_S_FILTER_CH2_EN 	_IOW(SINOROCK_FPGA_MAGIC,28,int) //设置应变CH2的滤波参数
#define IOCTRL_FPGA_PWM_S_FILTER_CH3_EN 	_IOW(SINOROCK_FPGA_MAGIC,29,int) //设置应变CH3的滤波参数
#define IOCTRL_FPGA_PWM_S_FILTER_CH4_EN 	_IOW(SINOROCK_FPGA_MAGIC,30,int) //设置应变CH4的滤波参数
//#define IOCTRL_FPGA_LED_TEST_EN 	_IOW(SINOROCK_FPGA_MAGIC,31,int) //发光二极管的测试
//预留的命令接口
#define IOCTRL_FPGA_WR_TEST31_EN 	_IOW(SINOROCK_FPGA_MAGIC,31,int) // TEST31
#define IOCTRL_FPGA_WR_TEST32_EN 	_IOW(SINOROCK_FPGA_MAGIC,32,int) // TEST32
#define IOCTRL_FPGA_WR_TEST33_EN 	_IOW(SINOROCK_FPGA_MAGIC,33,int) // TEST33
#define IOCTRL_FPGA_WR_TEST34_EN 	_IOW(SINOROCK_FPGA_MAGIC,34,int) // TEST34
#define IOCTRL_FPGA_WR_TEST35_EN 	_IOW(SINOROCK_FPGA_MAGIC,35,int) // TEST35
#define IOCTRL_FPGA_WR_TEST36_EN 	_IOW(SINOROCK_FPGA_MAGIC,36,int) // TEST36
#define IOCTRL_FPGA_WR_TEST37_EN 	_IOW(SINOROCK_FPGA_MAGIC,37,int) // TEST37
#define IOCTRL_FPGA_WR_TEST38_EN 	_IOW(SINOROCK_FPGA_MAGIC,38,int) // TEST38
#define IOCTRL_FPGA_WR_TEST39_EN 	_IOW(SINOROCK_FPGA_MAGIC,39,int) // TEST39
#define IOCTRL_FPGA_WR_TEST40_EN 	_IOW(SINOROCK_FPGA_MAGIC,40,int) // TEST40
//读操作：
#define IOCTRL_FPGA_BUS_TEST_EN 	_IOR(SINOROCK_FPGA_MAGIC,1,int)//总线测试
#define IOCTRL_FPGA_SAMPLE_END_FLAG_EN 	_IOR(SINOROCK_FPGA_MAGIC,2,int)//获取采样结束标准位
#define IOCTRL_FPGA_GET_BATTERY_EN	_IOR(SINOROCK_FPGA_MAGIC,3,int)//获取电量值
#define IOCTRL_FPGA_HIGH_DATA_LOW_EN 	_IOR(SINOROCK_FPGA_MAGIC,4,int)//获取高度低16位
#define IOCTRL_FPGA_HIGH_DATA_HIGH_EN 	_IOR(SINOROCK_FPGA_MAGIC,5,int)//获取高度高16位
#define IOCTRL_FPGA_GET_SAMPLE_DATA_A_CH1_EN 	_IOR(SINOROCK_FPGA_MAGIC,6,int)//获取加速度 CH1的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_A_CH2_EN 	_IOR(SINOROCK_FPGA_MAGIC,7,int)//获取加速度 CH2的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_A_CH3_EN 	_IOR(SINOROCK_FPGA_MAGIC,8,int)//获取加速度 CH3的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_A_CH4_EN 	_IOR(SINOROCK_FPGA_MAGIC,9,int)//获取加速度 CH4的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_S_CH1_EN 	_IOR(SINOROCK_FPGA_MAGIC,10,int)//获取应变 CH1的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_S_CH2_EN 	_IOR(SINOROCK_FPGA_MAGIC,11,int)//获取应变 CH2的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_S_CH3_EN 	_IOR(SINOROCK_FPGA_MAGIC,12,int)//获取应变 CH3的采集数据
#define IOCTRL_FPGA_GET_SAMPLE_DATA_S_CH4_EN 	_IOR(SINOROCK_FPGA_MAGIC,13,int)//获取应变 CH4的采集数据
#define IOCTRL_FPGA_GET_FIRMER_EN 	_IOR(SINOROCK_FPGA_MAGIC,14,int)//获取版本号
#define IOCTRL_FPGA_TEST16_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,16,int)//TEST16
#define IOCTRL_FPGA_TEST17_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,17,int)//TEST17
#define IOCTRL_FPGA_TEST18_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,18,int)//TEST18
#define IOCTRL_FPGA_TEST19_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,19,int)//TEST19
#define IOCTRL_FPGA_TEST20_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,20,int)//TEST20
#define IOCTRL_FPGA_TEST21_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,21,int)//TEST21
#define IOCTRL_FPGA_TEST22_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,22,int)//TEST22
#define IOCTRL_FPGA_TEST23_RD_EN 	_IOR(SINOROCK_FPGA_MAGIC,23,int)//TEST23

#define SINOROCK_CHIPNAME		"sinorockfpga"
//#define SMSC_MDIONAME		"smsc911x-mdio"
#define SINOROCK_DRV_VERSION	"2010-12-15"
#define DEVICE_NAME 	"sinorockfpga"
#define SINOROCKFPGA_MAJOR	201	// 0 为动态分派
MODULE_LICENSE("GPL");
MODULE_VERSION(SINOROCK_DRV_VERSION);
/*
#if USE_DEBUG > 0
static int debug = 16;
#else
static int debug = 3;
#endif

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");
*/

struct SinorockFpga_data
{
        //   void __iomem *ioaddr;
            unsigned long Fpga_ioaddr;
	    struct cdev cdev;	
	    spinlock_t dev_lock;
	    unsigned short  data_buf[32768*24];

}SinorockFpga_data_t;
//unsigned long Fpga_ioaddr;
struct SinorockFpga_data * SinorockFpga_dev;
static int sinorockfpga_major = SINOROCKFPGA_MAJOR;
//read data from  fpga
static inline u16 sinorockfpga_reg_read(/*struct SinorockFpga_data *pdata,*/ u32 reg)
{
 /*    u16  data;
     void *address=(void *)Fpga_ioaddr+reg;
     printk(KERN_INFO " read fpga data%d \n",data);
     data=ioread16(address);
     printk(KERN_INFO " read fpga data%d \n",data);
     return data;*/
     return 0;

}
//write data to fpga
static inline void sinorockfpga_reg_write(/*struct SinorockFpga_data *pdata,*/ u32 reg, u16 val)
{
 /*     void *address=(void *)Fpga_ioaddr+reg;
      iowrite16(val ,address);*/

}

//get sample data 
static inline void sinorockfpga_read_data(/*struct SinorockFpga_data *pdata, */ u32  count)
{
        int i=0;
        count=512*6;
	
	for(i=0;i<count;i++)
	{
	  //  pdata->data_buf[i]=sinorockfpga_reg_read(0);//index???
	}
	  
}
static int sinorockfpga_open(struct inode *inode, struct file *filp)
{

   //struct SinorockFpga_data *dev; /* device information */	
   //dev = container_of(inode->i_cdev, struct SinorockFpga_data, cdev);	
   //printk(KERN_INFO " open fpga \n");
   filp->private_data = SinorockFpga_dev; /* for other methods */
   return 0;
       
}

static int sinorockfpga_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret=0;
    int data=0;
    struct SinorockFpga_data *dev = filp->private_data; 
   // printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
    switch(cmd)
    {
       	case IOCTRL_FPGA_START_SAMPLE_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01*2);      
           break;
         }
       	case IOCTRL_FPGA_STOP_SAMPLE_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x02*2);        
           break;
         }
      	 case IOCTRL_FPGA_SET_SAMPLE_NUMBER_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x03*2);        
           break;
         }
	case IOCTRL_FPGA_SET_SAMPLE_RATE_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x04*2);        
           break;
         }
	case IOCTRL_FPGA_SET_DELAY_POINT_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x05*2);        
           break;
         }
	case IOCTRL_FPGA_SET_INIT_ADDR_FOR_READ_DATA_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x06*2);        
           break;
         }
	case IOCTRL_FPGA_START_BATTERY_SAMPLE_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x07*2);        
           break;
         }
	case IOCTRL_FPGA_TRIG_ENABLE_OR_DISABLE_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x08*2);        
           break;
         }
	case IOCTRL_FPGA_TRIGGER_POS_EN:
         {
          // printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x09*2);        
           break;
         }
	case IOCTRL_FPGA_TRIGGER_NEG_EN:
         {
           //printk(KERN_INFO " ioctl arg from user=%lx\n",arg);
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0A*2);        
           break;
         }
	case IOCTRL_FPGA_SET_CF_WIDE_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0B*2);        
           break;
         }
	case IOCTRL_FPGA_SET_CF_LOW_OR_HIGH_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0C*2);        
           break;
         }
	case IOCTRL_FPGA_SET_RELAY_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0D*2);        
           break;
         }
	case IOCTRL_FPGA_HIGH_CLEAR_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0E*2);        
           break;
         }
	case  IOCTRL_FPGA_A_AMP_CH1_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x0F*2);     
           break;
         }
	case IOCTRL_FPGA_A_AMP_CH2_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x010*2);        
           break;
         }
	case IOCTRL_FPGA_A_AMP_CH3_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x011*2);        
           break;
         }
	case IOCTRL_FPGA_A_AMP_CH4_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x012*2);        
           break;
         }
	case IOCTRL_FPGA_S_AMP_CH1_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x013*2);        
           break;
         }
	case IOCTRL_FPGA_S_AMP_CH2_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x014*2);        
           break;
         }
	case IOCTRL_FPGA_S_AMP_CH3_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x015*2);        
           break;
         }
	case IOCTRL_FPGA_S_AMP_CH4_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x016*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_A_FILTER_CH1_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x017*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_A_FILTER_CH2_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x018*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_A_FILTER_CH3_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x019*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_A_FILTER_CH4_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01A*2);        
           break;
         }
       	case IOCTRL_FPGA_PWM_S_FILTER_CH1_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01B*2);       
           break;
         }
	case IOCTRL_FPGA_PWM_S_FILTER_CH2_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01C*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_S_FILTER_CH3_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01D*2);        
           break;
         }
	case IOCTRL_FPGA_PWM_S_FILTER_CH4_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01E*2);        
           break;
         }
	case IOCTRL_FPGA_WR_TEST31_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x01F*2);       
           break;
         }
       	case IOCTRL_FPGA_WR_TEST32_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x020*2);      
           break;
         }
	case IOCTRL_FPGA_WR_TEST33_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x021*2);      
           break;
         }
        case IOCTRL_FPGA_WR_TEST34_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x022*2);      
           break;
         }
        case IOCTRL_FPGA_WR_TEST35_EN:
         {
           iowrite16(arg&0XFFFF,(void *)(dev->Fpga_ioaddr)+0x023*2);      
           break;
         }
        case IOCTRL_FPGA_BUS_TEST_EN:
         {
           data=ioread16((void *)(dev->Fpga_ioaddr)+0X01*2);
           __put_user(data,(int*)arg);
           break;
         }
        case IOCTRL_FPGA_SAMPLE_END_FLAG_EN:
         {
           data=ioread16((void *)(dev->Fpga_ioaddr)+0X02*2);
           __put_user(data,(int*)arg);
           break;
         }
	case IOCTRL_FPGA_GET_BATTERY_EN:
         {
           data=ioread16((void *)(dev->Fpga_ioaddr)+0X03*2);
           __put_user(data,(int*)arg);
           break;
         }
	case IOCTRL_FPGA_HIGH_DATA_LOW_EN:
         {
           data=ioread16((void *)(dev->Fpga_ioaddr)+0X04*2);
           __put_user(data,(int*)arg);
           break;
         }
	case IOCTRL_FPGA_HIGH_DATA_HIGH_EN:
         {
           data=ioread16((void *)(dev->Fpga_ioaddr)+0X05*2);
           __put_user(data,(int*)arg);
           break;
         }
       	case IOCTRL_FPGA_GET_FIRMER_EN:
         {
          data=ioread16((void *)(dev->Fpga_ioaddr)+0X0E*2);
           __put_user(data,(int*)arg);
           break;
         }
  	case IOCTRL_FPGA_TEST16_RD_EN:
         {
        //  printk(KERN_INFO " ioctl test 16\n");
          data=ioread16((void *)(dev->Fpga_ioaddr)+0X10*2);
           __put_user(data,(int*)arg);
           break;
         }
	case IOCTRL_FPGA_TEST17_RD_EN:
         {
          data=ioread16((void *)(dev->Fpga_ioaddr)+0X11*2);
          __put_user(data,(int*)arg);
           break;
         }
      default:
            return -EINVAL;
      

    }
   return ret;
}

static ssize_t sinorockfpga_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{     
      unsigned int data=0;
      unsigned int len=0,count=0,tmp=0;
      len=size/2;
      struct SinorockFpga_data *dev = filp->private_data; 
      if(len>32768*24)
        return -EFAULT; 
      //unsigned short *kbuf=kmalloc(len,GFP_KERNEL);
      //if(!kbuf){
     // printk(KERN_INFO " mem malloc err\n");
     // return -ENOMEM;
     // }
     // while(len--)
    // {
      // data=ioread16((void *)(dev->Fpga_ioaddr)+0X06*2);
       //data=((data&0xff)<<8)|((data&0xff00)>>8);
       //rmb();
       //printk(KERN_INFO " read fpga data=%d\n",data);
      // *(kbuf++)=data;//ioread16((void *)(dev->Fpga_ioaddr)+0X06*2);
     //}
  for(tmp=0;tmp<len;tmp++)
     {
       dev->data_buf[tmp]=ioread16((void *)(dev->Fpga_ioaddr)+0X06*2);
       //printk(KERN_INFO " read fpga data=%d:%d\n",tmp,dev->data_buf[tmp]);
     }
     count=(copy_to_user(buf,dev->data_buf,len*2));
    // printk(KERN_INFO " copy to user return=%d\n",count);
     if(count)
     {
      // kfree(kbuf);
       return -EFAULT;    
     }
     // sinorockfpga_reg_read(0);
    //data=ioread16((void *)(dev->Fpga_ioaddr)+0X18);
     // printk(KERN_INFO " read fpga data=%lx\n",data);
      //kfree(kbuf);
      return size;


}

static ssize_t sinorockfpga_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
      struct SinorockFpga_data *dev = filp->private_data; 
      //printk(KERN_INFO " write fpga \n");
     // sinorockfpga_reg_write(0,0xaa55);
      iowrite16(0XAA55,(void *)(dev->Fpga_ioaddr)+0x00);
      if(copy_from_user(dev->data_buf,buf,size))
           return -1;
      else
      {
         return size;
      }
      return 1;


}

static struct file_operations sinorockfpga_fops = {
    .owner  =   THIS_MODULE,    /* 这是一个宏，指向编译模块时自动创建的__this_module变量 */
    .open   =   sinorockfpga_open,     
    .ioctl   =  sinorockfpga_ioctl,
    .read   =   sinorockfpga_read,
    .write  =   sinorockfpga_write,
};


/*初始化并注册cdev*/
static void sinorockfpga_setup_cdev(struct SinorockFpga_data *dev, int index)
{
  int err, devno = MKDEV(sinorockfpga_major, index);
  cdev_init(&dev->cdev, &sinorockfpga_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &sinorockfpga_fops;
  err = cdev_add(&dev->cdev, devno, 1);
  if (err)
    printk(KERN_DEBUG " gpmc setup up cdev error\n ");
}


/* Entry point for loading the module */
static int __init sinorockfpga_init_module(void)
{
	int result = 0;
	int fpga_cs=SinorockFpga_GPMC_CS;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct resource *res;
	struct clk *l3ck;
       
	dev_t devno = MKDEV(sinorockfpga_major, 0);
   //     printk(KERN_INFO "sinorock init for fpga\n");
	/* 申请设备号*/
	if (sinorockfpga_major)
		result = register_chrdev_region(devno, 1, DEVICE_NAME);
	else  /* 动态申请设备号 */
	{
		result = alloc_chrdev_region(&devno, 0, 1, DEVICE_NAME);
	    	sinorockfpga_major = MAJOR(devno);
	}  
	if(result <0)
		return result;

        SinorockFpga_dev = kmalloc(sizeof(struct SinorockFpga_data), GFP_KERNEL);

	if (!SinorockFpga_dev)    /*申请失败*/
	{
    		result =  - ENOMEM;
    		//goto fail_malloc;
                printk(KERN_INFO "Failed to request mem for sinorockfpga dev\n");
                return result;
  	}
  	memset(SinorockFpga_dev, 0, sizeof(struct SinorockFpga_data));
      //  printk(KERN_INFO "memset sinorock fpga\n");
        
        l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);
	
 /*       //fpga_cs=SinorockFpga_GPMC_CS;
        int i=0;
        unsigned int regval;
        regval=gpmc_read_reg(GPMC_CONFIG);
        printk(KERN_INFO " gpmc config reg for fpga %08x\n",regval);
        regval|=(1<<1);
        gpmc_write_reg(GPMC_CONFIG,regval);
        regval=gpmc_cs_read_reg(fpga_cs,GPMC_CS_CONFIG7);
        //regval&=~(1<<6);
        regval=0x0F5F;//(1<<6);
        gpmc_cs_write_reg(fpga_cs,GPMC_CS_CONFIG7,regval);

       // regval=gpmc_cs_read_reg(fpga_cs,GPMC_CS_CONFIG1);
       // regval|=(1<<12);
       // gpmc_cs_write_reg(fpga_cs,GPMC_CS_CONFIG1,regval);
        
         for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(5,i*4);
        gpmc_cs_write_reg(fpga_cs,i*4,result);
        printk(KERN_INFO " gpmc cs5 config reg%d for fpga %08x\n",i+1,result);
        }
        regval=0x0F5F;//(1<<6);
        gpmc_cs_write_reg(fpga_cs,GPMC_CS_CONFIG7,regval);
         for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(0,i*4);
       // gpmc_cs_write_reg(fpga_cs,i*4,result);
        printk(KERN_INFO " gpmc cs0 config reg%d for fpga %08x\n",i+1,result);
        }
         for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(1,i*4);
       // gpmc_cs_write_reg(fpga_cs,i*4,result);
        printk(KERN_INFO " gpmc cs1 config reg%d for fpga %08x\n",i+1,result);
        }
         for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(2,i*4);
       // gpmc_cs_write_reg(fpga_cs,i*4,result);
        printk(KERN_INFO " gpmc cs2 config reg%d for fpga %08x\n",i+1,result);
        }
        for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(fpga_cs,i*4);
        printk(KERN_INFO " gpmc cs3 config reg%d for fpga %08x\n",i+1,result);
        }*/
        //write config_i
	if (gpmc_cs_request(fpga_cs, SZ_16M, &cs_mem_base) < 0) 
	{
		printk(KERN_INFO "Failed to request GPMC mem for fpga\n");
		return - ENOMEM;
	}//0x01000000

      //  cs_mem_base=0X18000000;       
	//printk(KERN_INFO " GPMC mem for fpga: %0lx\n",cs_mem_base);
	//res->start=cs_mem_base;
	//res->end=cs_mem_base+1024-1;
	int res_size=SZ_16M;
	if (!request_mem_region(cs_mem_base, res_size, SINOROCK_CHIPNAME)) 
	{
	       printk(KERN_INFO "Failed to request_mem_region for fpga\n");
	       return -EBUSY;
		
	}
 /*       int i=0;
        for(i=0;i<7;i++)
        {
        result=gpmc_cs_read_reg(fpga_cs,i*4);
        printk(KERN_INFO " gpmc cs3 config reg%d for fpga %08x\n",i+1,result);
        }*/
       // SinorockFpga_dev->ioaddr=ioremap_nocache(res->start, res_size);
        SinorockFpga_dev->Fpga_ioaddr=ioremap(cs_mem_base, res_size);
	//printk(KERN_INFO " GPMC mem for fpga ioremap:%0lx\n",SinorockFpga_dev->Fpga_ioaddr);
	 if (SinorockFpga_dev->Fpga_ioaddr == NULL) 
        {
               printk(KERN_INFO "Failed to request_mem_region for fpga\n");
	       return -ENOMEM;
		 
	}
        sinorockfpga_setup_cdev(SinorockFpga_dev, 0);
	return 0;

	
}

/* entry point for unloading the module */
static void __exit sinorockfpga_cleanup_module(void)
{
    if (SinorockFpga_dev)
    	{
    	    kfree(SinorockFpga_dev);
    	}
    unregister_chrdev(sinorockfpga_major,DEVICE_NAME);
    iounmap((void __iomem *)SinorockFpga_dev->Fpga_ioaddr);
    release_mem_region(SinorockFpga_dev->Fpga_ioaddr,1024);
	
}

module_init(sinorockfpga_init_module);
module_exit(sinorockfpga_cleanup_module);

