/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/apds9930.c - APDS9930 ALS/PS driver
 * 
 * Author: Chanson Tan <tanxs@gionee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/hwmsen_helper.h>

#include <asm/atomic.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6577
#include <mach/mt6577_devs.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_pm_ldo.h>
#endif

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#define POWER_NONE_MACRO -1

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6577
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps_apds9930.h>
#include "gn_avago_apds9930.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9930_DEV_NAME     "APDS9930"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_INFO(fmt, args...)   printk(KERN_INFO APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)              
/******************************************************************************
 * extern functions
*******************************************************************************/

/*#ifdef MT6577*/
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
	extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
	extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
	extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);
	


/*----------------------------------------------------------------------------*/
static struct i2c_client *apds9930_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id apds9930_i2c_id[] = {{APDS9930_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_APDS9930={ I2C_BOARD_INFO("APDS9930", APDS_I2C_SLAVE_ADDR)};
static int apds9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9930_i2c_remove(struct i2c_client *client);
static int apds9930_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9930_i2c_resume(struct i2c_client *client);

static struct apds9930_priv *g_apds9930_ptr = NULL;

static int apds9930_local_init(void);
static int apds9930_remove(void);
static int apds9930_init_flag = 0;    //0<==>OK,-1<==>fail

static struct sensor_init_info apds9930_init_info = {
    .name = "psensor_apds9930",
    .init = apds9930_local_init,
    .uninit = apds9930_remove,
};

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf;
    int ret = 0;
    struct i2c_client client_read = *client;

    client_read.addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
    ret = i2c_master_send(&client_read, (const char*)&buf, 1<<8 | 1);
    if (ret < 0) {
        APS_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
    client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

static struct PS_CALI_DATA_STRUCT ps_cali={{0,0,0},};
/*----------------------------------------------------------------------------*/
typedef enum 
{
    CMC_BIT_ALS = 1,
    CMC_BIT_PS  = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct apds9930_i2c_addr 
{    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct apds9930_priv 
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct apds9930_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    trace;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver apds9930_i2c_driver = {	
	.probe      = apds9930_i2c_probe,
	.remove     = apds9930_i2c_remove,
	.detect     = apds9930_i2c_detect,
	.suspend    = apds9930_i2c_suspend,
	.resume     = apds9930_i2c_resume,
	.id_table   = apds9930_i2c_id,
	.driver = {
		.name           = APDS9930_DEV_NAME,
	},
};
static int apds9930_get_ps_value(struct apds9930_priv *obj, u16 ps);
static int apds9930_get_als_value(struct apds9930_priv *obj, u16 ps);
static struct apds9930_priv *apds9930_obj = NULL;

//static struct platform_driver apds9930_alsps_driver;
/*----------------------------------------------------------------------------*/
int apds9930_get_addr(struct alsps_hw *hw, struct apds9930_i2c_addr *addr)
{
	if(!hw || !addr) {
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void apds9930_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;
	if(hw->power_id != POWER_NONE_MACRO) {
		if(power_on == on) {
			APS_LOG("ignore power control: %d\n", on);
		} else if(on) {
			if(!hwPowerOn(hw->power_id, hw->power_vol, "APDS9930")) {
				APS_ERR("power on fails!!\n");
			}
		} else {
			if(!hwPowerDown(hw->power_id, "APDS9930")) {
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long apds9930_enable_als(struct i2c_client *client, int enable)
{
		struct apds9930_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		long res = 0;
		u8 buffer[1];
		u8 reg_value[1];
		uint32_t testbit_PS;
	
		if(client == NULL){
			APS_INFO("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}	
		testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if(enable){
			if(testbit_PS) {	
			databuf[0] = APDS9930_CMM_ENABLE;	
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
	    		if(res <= 0) {
					goto EXIT_ERR;
			}
			} else {
			databuf[0] = APDS9930_CMM_ENABLE;	
			databuf[1] = 0x2B;
			res = i2c_master_send(client, databuf, 0x2);
		    	if(res <= 0){
					goto EXIT_ERR;
		    	}

			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_INFO("apds9930 power on\n");
		} else {	
			if(testbit_PS) {
			databuf[0] = APDS9930_CMM_ENABLE;	
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
		        if(res <= 0) {
					goto EXIT_ERR;
				}
			} else {
			databuf[0] = APDS9930_CMM_ENABLE;	
			databuf[1] = 0x29;
			res = i2c_master_send(client, databuf, 0x2);
		    	if(res <= 0) {
					goto EXIT_ERR;
				}
			}
			atomic_set(&obj->als_deb_on, 0);
			APS_INFO("apds9930 power off\n");
		}
		
		return 0;
		
EXIT_ERR:
		APS_ERR("apds9930_enable_als fail\n");
		return res;
}

static apds9930_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

   APS_LOG("apds9930_WriteCalibration  %d,%d,%d\n",data_cali->close,data_cali->far_away,data_cali->valid);

   if(data_cali->valid ==1) {
       ps_cali.close = data_cali->close; 
       ps_cali.far_away = data_cali->far_away;
       ps_cali.valid = 1;
       
       hwmsen_write_byte(apds9930_obj->client, 0x8A, (ps_cali.close) & 0x00ff);
       hwmsen_write_byte(apds9930_obj->client, 0x8B, ((ps_cali.close)>>8) & 0x07);
       hwmsen_write_byte(apds9930_obj->client, 0x88, (ps_cali.far_away)& 0x00ff);
       hwmsen_write_byte(apds9930_obj->client, 0x89, ((ps_cali.far_away)>>8)&0x07);
   } else {
       ps_cali.valid = 0; 
       hwmsen_write_byte(apds9930_obj->client, 0x8A, PS_THRES_UP_0_VALUE);
       hwmsen_write_byte(apds9930_obj->client, 0x8B, PS_THRES_UP_1_VALUE);
       hwmsen_write_byte(apds9930_obj->client, 0x88, PS_THRES_LOW_0_VALUE);
       hwmsen_write_byte(apds9930_obj->client, 0x89, PS_THRES_LOW_1_VALUE);
   }
	  
}

static long apds9930_enable_ps(struct i2c_client *client, int enable)
{
	struct apds9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	u8 testbit_ALS;

	if(client == NULL) {
		APS_INFO("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
	if(enable) {
		if(testbit_ALS){
		       databuf[0] = APDS9930_CMM_ENABLE;    
		       databuf[1] = 0x0F;
		       res = i2c_master_send(client, databuf, 0x2);
		          if(res <= 0) {
				goto EXIT_ERR;
			   }
	       } else {
		       databuf[0] = APDS9930_CMM_ENABLE;    
		       databuf[1] = 0x0D;
		       res = i2c_master_send(client, databuf, 0x2);
		          if(res <= 0) {
				goto EXIT_ERR;
			   }
		}
              atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_INFO("apds9930 power on\n");
		if(0 == obj->hw->polling_mode_ps) {
			if(1 == ps_cali.valid) {
				databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
			} else {
				databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(PS_THRES_LOW_VALUE & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((PS_THRES_LOW_VALUE & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(PS_THRES_UP_VALUE & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
				databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((PS_THRES_UP_VALUE & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0) {
					goto EXIT_ERR;
				}
		
			}
                     databuf[0] = APDS9930_CMM_Persistence;
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0) {
				goto EXIT_ERR;
			}
			if(testbit_ALS) {
			databuf[0] = APDS9930_CMM_ENABLE;    
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0) {
					goto EXIT_ERR;
			}
			} else {
			databuf[0] = APDS9930_CMM_ENABLE;    
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0) {
					goto EXIT_ERR;
				}
			}
		
		//	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
        }
		
	} else {
	if(testbit_ALS) {
		databuf[0] = APDS9930_CMM_ENABLE;    
		databuf[1] = 0x2B;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0) {
				goto EXIT_ERR;
		}
		} else {
		databuf[0] = APDS9930_CMM_ENABLE;    
		databuf[1] = 0x29;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0) {
				goto EXIT_ERR;
		}
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_LOG("apds9930 power off\n");

		if(0 == obj->hw->polling_mode_ps) {
			cancel_work_sync(&obj->eint_work);
//			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		}
	}

	return 0;
	
EXIT_ERR:
	APS_ERR("apds9930_enable_ps fail\n");
	return res;
}
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
static int apds9930_check_and_clear_intr(struct i2c_client *client) 
{
	struct apds9930_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];
	buffer[0] = APDS9930_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0) {
		goto EXIT_ERR;
	}
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20)) {
		res = 0;
		intp = 1;
	}
       if(0 == res) { 
              buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
    
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0) {
			goto EXIT_ERR;
		} else {
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("apds9930_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/
void apds9930_eint_func(void)
{
	struct apds9930_priv *obj = g_apds9930_ptr;
	if(!obj) {
		return;
	}
	
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
int apds9930_setup_eint(struct i2c_client *client)
{
	struct apds9930_priv *obj = i2c_get_clientdata(client);        

	g_apds9930_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, apds9930_eint_func, 0);

	//mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}
#endif
/*----------------------------------------------------------------------------*/

static int apds9930_init_client(struct i2c_client *client)
{
	struct apds9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	   databuf[0] = APDS9930_CMM_ENABLE; 
          databuf[1] = 0x01;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		  goto EXIT_ERR;
	   }
	
	   databuf[0] = APDS9930_CMM_ATIME;    
	   databuf[1] = 0xEE;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		   goto EXIT_ERR;
	   }

	   databuf[0] = APDS9930_CMM_PTIME;    
	   databuf[1] = 0xFF;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		   goto EXIT_ERR;
	   }

          databuf[0] = APDS9930_CMM_WTIME;    
	   databuf[1] = 0xF6;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		   goto EXIT_ERR;
           }
	   databuf[0] = APDS9930_CMM_Persistence;
	   databuf[1] = 0x00;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		   goto EXIT_ERR;
           }

	   databuf[0] = APDS9930_CMM_ENABLE;	
	   databuf[1] = 0x01 | 0x20;
	   res = i2c_master_send(client, databuf, 0x2);
	   if(res <= 0) {
		   goto EXIT_ERR;
	   }

    	  databuf[0] = APDS9930_CMM_CONFIG;    
    	  databuf[1] = 0x00;
    	  res = i2c_master_send(client, databuf, 0x2);
    	  if(res <= 0) {
		   goto EXIT_ERR;
          }
         databuf[0] = APDS9930_CMM_POFFSET;
         databuf[1] = 0x00;
         res = i2c_master_send(client, databuf, 0x2);
         if(res <= 0) {
                 goto EXIT_ERR;
          }

    	 databuf[0] = APDS9930_CMM_PPCOUNT;    
    	 databuf[1] = APDS9930_CMM_PPCOUNT_VALUE;
    	 res = i2c_master_send(client, databuf, 0x2);
    	 if(res <= 0) {
	    	  goto EXIT_ERR;
    	 }

    	 databuf[0] = APDS9930_CMM_CONTROL;
    	 databuf[1] = 0x20;
    	 res = i2c_master_send(client, databuf, 0x2);
    	 if(res <= 0) {
	    	 goto EXIT_ERR;
    	 }
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    	 if(res = apds9930_setup_eint(client)) {
	    	APS_ERR("setup eint: %d\n", res);
	    	return res;
    	 }
    	 if(res = apds9930_check_and_clear_intr(client)) {
	    	APS_ERR("check/clear intr: %d\n", res);
    	 }
#endif
	return APDS9930_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int apds9930_read_als(struct i2c_client *client, u16 *data)
{
	struct apds9930_priv *obj = i2c_get_clientdata(client);	 
	int c0_value ; 
       int c1_value ;
       int c0_value_lo = 0;
       int c0_value_hi = 0;
       int c1_value_lo = 0;
       int c1_value_hi = 0;	 
	u32 c0_nf, c1_nf;
	u16 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	u16 als_value;
	int res = 0;
	u8 reg_value[1];
	
	if(client == NULL) {
		APS_INFO("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}       
       if (hwmsen_read_byte_sr(client, APDS9930_CMM_C1DATA_L, &c1_value_lo)) {
            APS_ERR("reads als data (ch1 lo) = %d\n", c1_value_lo);
            return -EFAULT;
        }
       if (hwmsen_read_byte_sr(client, APDS9930_CMM_C1DATA_H, &c1_value_hi)) {
            APS_ERR("reads aps data (ch1 hi) = %d\n", c1_value_hi);
            return -EFAULT;
        }
       c1_value = (c1_value_hi * 256) + c1_value_lo;
       c1_nf = obj->als_modulus*c1_value;
       APS_LOG("c1_value_hi=%x c1_value_lo=%x\n",c1_value_hi,c1_value_lo);


       if (hwmsen_read_byte_sr(client, APDS9930_CMM_C0DATA_L, &c0_value_lo)) {
           APS_ERR("reads als data (ch0 lo) = %d\n", c0_value_lo);
           return -EFAULT;
        }
       if (hwmsen_read_byte_sr(client, APDS9930_CMM_C0DATA_H, &c0_value_hi)) {
           APS_ERR("reads als data (ch0 hi) = %d\n", c0_value_hi);
           return -EFAULT;
        }
       c0_value = (c0_value_hi * 256) + c0_value_lo;
       c0_nf = obj->als_modulus*c0_value;
       APS_LOG("c0_value_hi=%x c0_value_lo=%x\n",c0_value_hi,c0_value_lo);

	if((c0_value > c1_value) &&(c0_value < 50000)) {  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;
	if(atio<30) {
		*data = (13*c0_nf - 24*c1_nf)/10000;
	} else if(atio>= 30 && atio<38) { 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	} else if(atio>= 38 && atio<45) { 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	} else if(atio>= 45 && atio<54) { 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	} else
		*data = 0;
        }
	else if (c0_value > 50000) {
		*data = 65534;
	}
	else if(c0_value == 0) {
              *data = 0;
       } else {
		APS_INFO("als_value is invalid!!\n");
		return -1;
	}	
       if((*data > 0) && (*data < 28)) {
              *data = 3 ; 
        }
       APS_INFO("als_value_lux = %d\n", *data);
	return 0;	 

	
EXIT_ERR:
	APS_ERR("apds9930_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int apds9930_get_als_value(struct apds9930_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
       int lastidx = 0;

       if(als < obj->hw->als_level[lastidx]+2 && als > obj->hw->als_level[lastidx]-2) {
                       idx = lastidx;
       } else {
	       for(idx = 0; idx < obj->als_level_num; idx++) {
		     if(als < obj->hw->als_level[idx]) {
                       lastidx = idx;
			  break;
		      }
	         }
        }
	
	if(idx >= obj->als_value_num) {
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on)) {

		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt)) {
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on)) {
			invalid = 1;
		}
	}

	if(!invalid) {
		APS_LOG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	} else {
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long apds9930_read_ps(struct i2c_client *client, u16 *data)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);    
    u16 ps_value;    
    u16 buffer[1];     
    u16 ps_value_low[1];
    long res = 0;
    int psval_lo = 0;
    int psval_hi = 0;
    int psdata = 0;

    if(client == NULL) {
	APS_INFO("CLIENT CANN'T EQUL NULL\n");
	return -1;
     }

    if (hwmsen_read_byte_sr(client, APDS9930_CMM_PDATA_L, &psval_lo)) {
        APS_ERR("reads aps data = %d\n", psval_lo);
        return -EFAULT;
     }

    if (hwmsen_read_byte_sr(client, APDS9930_CMM_PDATA_H, &psval_hi)) {
        APS_ERR("reads aps hi data = %d\n", psval_hi);
        return -EFAULT;
     }

     psdata = ((psval_hi & 7) * 256) + psval_lo;
     *data = psdata;

     return 0;
}
/*------------------------------------------------------------------------
 sysfs attributes
-- ---------------------------------------------------------------------*/
static ssize_t apds9930_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n",
       atomic_read(&apds9930_obj->i2c_retry),atomic_read(&apds9930_obj->als_debounce),
       atomic_read(&apds9930_obj->ps_mask),atomic_read(&apds9930_obj->ps_thd_val_low),
       atomic_read(&apds9930_obj->ps_thd_val_high),atomic_read(&apds9930_obj->ps_debounce));
	return res;
}

static ssize_t apds9930_store_config(struct device_driver *ddri, char *buf, size_t count)
{
    int retry, als_deb, ps_deb, mask, thres;
    if(!apds9930_obj) {
        APS_ERR("apds9930_obj is null!!\n");
        return 0;
    }

    if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb)) {
        atomic_set(&apds9930_obj->i2c_retry, retry);
        atomic_set(&apds9930_obj->als_debounce, als_deb);
        atomic_set(&apds9930_obj->ps_mask, mask);
        atomic_set(&apds9930_obj->ps_debounce, ps_deb);
    } else {
        APS_ERR("invalid content: '%s', length = %d\n", buf, count);
     }
    return count;
}

static ssize_t apds9930_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&apds9930_obj->trace));
	return res;
}

static ssize_t apds9930_store_trace(struct device_driver *ddri, char *buf, size_t count)
{
	int trace;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace)) {
		atomic_set(&apds9930_obj->trace, trace);
	} else {
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
      return count;
}

static ssize_t apds9930_show_reg(struct device_driver *ddri, char *buf)
{
    int i=0;
    u8 addr = 0x80;
    u8 dat = 0;
    int regdata[1024]={0};
    int len=0;

    if(!apds9930_obj) {
        APS_ERR("apds9930_obj is null!!\n");
        return 0;
    }

    for (i = 0; i < 20; i++) {
        hwmsen_read_byte_sr(apds9930_i2c_client,addr,&regdata[i]);
        dat = regdata[i];
        len+=snprintf(buf+len,16,"Reg addr=%x ",addr);
        len+=snprintf(buf+len,16,"Reg regdata=%x\n",dat);
        addr++;
        if (addr == 0x84)
            addr += 0x04;
        if (addr == 0x90)
            addr += 0x03;
        if (addr > 0x99)
            break;
    }

    return len;
}

static ssize_t apds9930_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	u8 dat = 0;

	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");	
        return 0;
	}
	
    if(res = apds9930_read_als(apds9930_obj->client, &apds9930_obj->als)) {
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	} else {
		dat = apds9930_obj->als;
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
	}
}

static ssize_t apds9930_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	u8 dat=0;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	if(res = apds9930_read_ps(apds9930_obj->client, &apds9930_obj->ps)) {
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	} else {
		dat = apds9930_get_ps_value(apds9930_obj, apds9930_obj->ps);
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
	}
}

static ssize_t apds9930_show_ps_raw(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    int dat=0;
    int data = -1;
    int err = 0 ;
    
    if(!apds9930_obj) {
        APS_ERR("apds9930_obj is null!!\n");
        return 0;
    }
    
    if(err = apds9930_enable_ps(apds9930_obj->client, true)) {
        APS_ERR("enable ps fail: %d\n", err);
        return -1;
    }
    
    set_bit(CMC_BIT_PS, &apds9930_obj->enable);
    msleep(300);
    
    if(res = apds9930_read_ps(apds9930_obj->client, &apds9930_obj->ps)) {
        snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else {   
        dat = apds9930_obj->ps & 0x80;
        data = dat & 0x0000FFFF;
        dat = apds9930_get_ps_value(apds9930_obj, apds9930_obj->ps);
        data = ((dat<<16) & 0xFFFF0000) | data;   
    }
    
    msleep(50);
    if(err = apds9930_enable_ps(apds9930_obj->client, false)) {
        APS_ERR("disable ps fail: %d\n", err);
        return -1;
    }
    clear_bit(CMC_BIT_PS, &apds9930_obj->enable);
    
    return snprintf(buf, PAGE_SIZE, "%08X\n", data);
}


static ssize_t apds9930_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	if(apds9930_obj->hw) {
              len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
		apds9930_obj->hw->i2c_num, apds9930_obj->hw->power_id, apds9930_obj->hw->power_vol);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&apds9930_obj->als_suspend), atomic_read(&apds9930_obj->ps_suspend));
       return len;
}

/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct apds9930_priv *obj, const char* buf, size_t count,
							 u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len) {
		while((cur < end) && IS_SPACE(*cur)) {
			cur++;
		}

		if(1 != sscanf(cur, "%d", &data[idx])) {
			break;
		}

		idx++;
		while((cur < end) && !IS_SPACE(*cur)) {
			cur++;
		}
	}
	
    return idx;
}

static ssize_t apds9930_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < apds9930_obj->als_level_num; idx++) {
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", apds9930_obj->hw->als_level[idx]);
	}
	
       len += snprintf(buf+len, PAGE_SIZE-len, "\n");
       return len;
}

static ssize_t apds9930_store_alslv(struct device_driver *ddri, char *buf, size_t count)
{
	struct apds9930_priv *obj;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	} else if(!strcmp(buf, "def")) {
		memcpy(apds9930_obj->als_level, apds9930_obj->hw->als_level, sizeof(apds9930_obj->als_level));
	} else if(apds9930_obj->als_level_num != read_int_from_buf(apds9930_obj, buf, count,
			apds9930_obj->hw->als_level, apds9930_obj->als_level_num)){
		APS_ERR("invalid format: '%s'\n", buf);
	}
	
    return count;
}

static ssize_t apds9930_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	}

	for(idx = 0; idx < apds9930_obj->als_value_num; idx++) {
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", apds9930_obj->hw->als_value[idx]);
	}
	
       len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}

static ssize_t apds9930_store_alsval(struct device_driver *ddri, char *buf, size_t count)
{
	if(!apds9930_obj) {
		APS_ERR("apds9930_obj is null!!\n");
		return 0;
	} else if(!strcmp(buf, "def")) {
		memcpy(apds9930_obj->als_value, apds9930_obj->hw->als_value, sizeof(apds9930_obj->als_value));
	} else if(apds9930_obj->als_value_num != read_int_from_buf(apds9930_obj, buf, count,
			apds9930_obj->hw->als_value, apds9930_obj->als_value_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	
    return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	    S_IWUSR | S_IRUGO, apds9930_show_als,     NULL);
static DRIVER_ATTR(ps,	    S_IWUSR | S_IRUGO, apds9930_show_ps,      NULL);
static DRIVER_ATTR(alsps,   S_IWUSR | S_IRUGO, apds9930_show_ps_raw,  NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, apds9930_show_config,  apds9930_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, apds9930_show_alslv,   apds9930_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, apds9930_show_alsval,  apds9930_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, apds9930_show_trace,   apds9930_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, apds9930_show_status,  NULL);
static DRIVER_ATTR(reg,	    S_IWUSR | S_IRUGO, apds9930_show_reg,     NULL);

/*----------------------------------------------------------------------------*/

static struct device_attribute *apds9930_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
       &driver_attr_alsps,
	&driver_attr_config,
       &driver_attr_alslv,
       &driver_attr_alsval,
       &driver_attr_trace,		/*trace log*/
	&driver_attr_status,
	&driver_attr_reg,
       
};

static int apds9930_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9930_attr_list)/sizeof(apds9930_attr_list[0]));
	if (driver == NULL) {
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++) {
		if(err = driver_create_file(driver, apds9930_attr_list[idx])) {
			APS_ERR("driver_create_file (%s) = %d\n", apds9930_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
	
static int apds9930_delete_attr(struct device_driver *driver)
{
      int idx ,err = 0;
      int num = (int)(sizeof(apds9930_attr_list)/sizeof(apds9930_attr_list[0]));

      if (!driver) {
        return -EINVAL;
       }
	
      for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, apds9930_attr_list[idx]);
       }

	return err;
}
static int apds9930_get_ps_value(struct apds9930_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp = 1;
	 u16 temp_ps[1];
	 
	 APS_LOG("apds9930_get_ps_value ps = %x \n", ps);
#if defined(GN_PS_NEED_CALIBRATION)
	if(ps_cali.valid == 1) {
			if(ps >ps_cali.close) {
				val = 0;  /*close*/
				val_temp = 0;
			} else if(ps <ps_cali.far_away) {
				val = 1;  /*far away*/
				val_temp = 1;
			}  else
				val = val_temp;

        } else {

			if(ps > atomic_read(&obj->ps_thd_val_high)) {
				val = 0;  /*close*/
				val_temp = 0;
			}
			else if(ps < atomic_read(&obj->ps_thd_val_low)) {
				val = 1;  /*far away*/
				val_temp = 1;
			}
			else
			       val = val_temp;	
     }
#else
			if(ps > atomic_read(&obj->ps_thd_val_high)) {
				val = 0;  /*close*/
				val_temp = 0;
			} else if(ps < atomic_read(&obj->ps_thd_val_low)) {
				val = 1;  /*far away*/
				val_temp = 1;
			} else
			       val = val_temp;

#endif
	
	if(atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if(1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on)) {
			invalid = 1;
		}
	} else if (obj->als > 10000) {
		APS_INFO("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid) {
		return val;
	} else {
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
static void apds9930_eint_work(struct work_struct *work)
{
	struct apds9930_priv *obj = (struct apds9930_priv *)container_of(work, struct apds9930_priv, eint_work);
    int err;
    u8 buf;
    hwm_sensor_data sensor_data;

    APS_FUN();
    memset(&sensor_data, 0, sizeof(sensor_data));
    if((err = apds9930_check_and_clear_intr(obj->client))) {
		APS_ERR("apds9930_eint_work check intrs: %d\n", err);
    } else {
        apds9930_read_ps(obj->client, &obj->ps);
        sensor_data.values[0] = apds9930_get_ps_value(obj, obj->ps);
        APS_LOG("apds9930 get ps value_apds9930 eint work  = %d \n",sensor_data.values[0]);

        if(sensor_data.values[0] == 0)    { //set the ps_threshold dynamiclly  
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_LOW,0xff)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }

            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_HIGH,0x07)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
#if defined(GN_PS_NEED_CALIBRATION)
            if(ps_cali.valid == 0) {                                               
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_LOW,PS_THRES_LOW_0_VALUE)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
            
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_HIGH,PS_THRES_LOW_1_VALUE)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
            } else if(ps_cali.valid == 1) {
                if(hwmsen_write_byte(obj->client, APDS9930_CMM_INT_LOW_THD_LOW, (ps_cali.far_away) & 0x00ff)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
                
                if(hwmsen_write_byte(obj->client, APDS9930_CMM_INT_LOW_THD_HIGH, ((ps_cali.far_away)>>8) & 0x07)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
              }
#else
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_LOW,PS_THRES_LOW_0_VALUE)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
           
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_HIGH,PS_THRES_LOW_1_VALUE)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
#endif
        } else if(sensor_data.values[0] == 1) {
#if defined(GN_PS_NEED_CALIBRATION)
            if(ps_cali.valid == 0) {
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_LOW,PS_THRES_UP_0_VALUE)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
            
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_HIGH,PS_THRES_UP_1_VALUE)) {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }
            } else if(ps_cali.valid == 1) {
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_LOW, ps_cali.close & 0x00ff)) {    
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  }    
                
                if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_HIGH, ((ps_cali.close)>> 8) & 0x07)) {    
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                  } 
             }
#else
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_LOW,PS_THRES_UP_0_VALUE)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
            
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_HIGH_THD_HIGH,PS_THRES_UP_1_VALUE)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
#endif
            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_LOW,0x00)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }

            if(hwmsen_write_byte(obj->client,APDS9930_CMM_INT_LOW_THD_HIGH,0x00)) {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
              }
        }
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)) {
            APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
        }
    }
      // mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
}
#endif

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int apds9930_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9930_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int apds9930_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


static int apds9930_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
    int ret=0;
    int i=0;
    u16 data[COUNT];
    int data_total=0;
    ssize_t len = 0;
    int noise = 0;
    int max = 0;
    int buf;
    int err = 0;
    int j = 0;
    hwm_sensor_data sensor_data;

    if(!apds9930_obj) { 
        APS_ERR("apds9930_obj is null!!\n");
        len = sprintf(buf, "apds9930_obj is null\n");
        goto report_value;
    }

    for (i = 0; i < COUNT; i++) {
   
        if (max++ > 50) {
            ps_cali.valid = 0;
            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;
            
            goto report_value;
        }
        
        mdelay(50);
        ret=apds9930_read_ps(apds9930_obj->client,&data[i]);

        APS_LOG("the ps value is %d \n", data[i]);

        if (ret < 0) {
            i--;
            continue;
         }

        data_total+= data[i];
        if(data[i] == 0) {
            j++;
         }
    }
    
    if(data_total == 0) {
        ps_data_cali->close = NOISE_HIGH;
        ps_data_cali->far_away = NOISE_LOW;
        ps_data_cali->valid = 1;

        ps_cali.close = NOISE_HIGH;
        ps_cali.far_away = NOISE_LOW;
        ps_cali.valid = 1;
    } else {
        noise=data_total/(COUNT-j);
        APS_LOG("get_noise_value  %d,", noise);
        if(noise > NOISE_MAX) {
            ps_cali.valid = 0;
            ps_cali.close = 0;
            ps_cali.valid = 0;

            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;
         } else {
            ps_data_cali->close = noise + NOISE_HIGH; 
            ps_data_cali->far_away = noise + NOISE_LOW;
            ps_data_cali->valid = 1;

            ps_cali.close = noise + NOISE_HIGH;
            ps_cali.far_away = noise + NOISE_LOW;
            ps_cali.valid = 1;
          }
        
    }
        APS_LOG("apds9930_read_data_for_cali %d,%d,%d\t",ps_cali.close, ps_cali.far_away,ps_cali.valid);

report_value:
    sensor_data.values[0] = 1;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)) {
        APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
    }
  
   return 0;	 	

}


/*----------------------------------------------------------------------------*/
static long apds9930_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct apds9930_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	
#if defined(GN_PS_NEED_CALIBRATION)
    struct PS_CALI_DATA_STRUCT ps_cali_temp;
#endif
	APS_FUN();
       APS_LOG("apds9930 cmd = %d \n", cmd);
	switch (cmd) {
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))) {
                APS_ERR("alsps set ps mode copy from user fail\n"); 
				err = -EFAULT;
				goto err_out;
			}
			if(enable) {
				if(err = apds9930_enable_ps(obj->client, true)) {
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			} else {
				if(err = apds9930_enable_ps(obj->client, false)) {
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable))) {
                            APS_ERR("alsps get ps mode_copy to user fail\n"); 
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = apds9930_read_ps(obj->client, &obj->ps)) {
                            APS_ERR("apds9930 read ps fail: %d\n", err);
				goto err_out;
			}
			
			dat = apds9930_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
                            APS_ERR("alsps get ps data_copy to user fail\n"); 
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = apds9930_read_ps(obj->client, &obj->ps)) {
                            APS_ERR("apds9930 read ps fail: %d\n", err);
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
                            APS_ERR("alsps get ps raw data_copy to user fail\n"); 
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))) {
                            APS_ERR("alsps set als mode_copy from user fail\n"); 
				err = -EFAULT;
				goto err_out;
			}
			if(enable) {
				if(err = apds9930_enable_als(obj->client, true)) {
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			} else {
				if(err = apds9930_enable_als(obj->client, false)) {
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable))) {
                            APS_ERR("alsps get als mode_copy to user fail\n");
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = apds9930_read_als(obj->client, &obj->als)) {
                            APS_ERR("apds9930 read als fail: %d\n", err); 
				goto err_out;
			}

			dat = apds9930_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
                            APS_ERR("alsps get als data_copy to user fail\n");
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = apds9930_read_als(obj->client, &obj->als)) {
                            APS_ERR("apds9930 read als fail: %d\n", err);
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}              
			break;

#if defined(GN_PS_NEED_CALIBRATION)
		case ALSPS_SET_PS_CALI:
			dat = (void __user*)arg;
			if(dat == NULL) {
				APS_LOG("dat == NULL\n");
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&ps_cali_temp, dat, sizeof(ps_cali_temp))) {
				APS_ERR("alsps set ps cali_copy from user fail\n");
				err = -EFAULT;
				break;	  
			}
			apds9930_WriteCalibration(&ps_cali_temp);
			APS_LOG(" ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close,ps_cali_temp.far_away,ps_cali_temp.valid);
			break;
            case ALSPS_GET_PS_CALI:
                     if(err = apds9930_enable_ps(obj->client, true)) {
                            APS_ERR("ads9930 ioctl enable ps fail: %d\n", err);
                        }
            
                     msleep(200);
                            err = apds9930_read_data_for_cali(obj->client, &ps_cali_temp);
                     if(err) {
                            goto err_out;
                        }
                     if(err = apds9930_enable_ps(client, false)) {
                            APS_ERR("apds9930 ioctl disable ps fail: %d\n", err);
                        }
            
                     if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp))) {
                            APS_ERR("alsps get ps cali_copy from user fail\n");
                            err = -EFAULT;
                            goto err_out;
                        }              
             APS_LOG("apds9930 ALSPS_GET_PS_CALI %d,%d,%d\t",ps_cali_temp.close, ps_cali_temp.far_away,ps_cali_temp.valid);
             break;
#endif
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}
err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations apds9930_fops = {
	.owner = THIS_MODULE,
	.open = apds9930_open,
	.release = apds9930_release,
	.unlocked_ioctl = apds9930_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice apds9930_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &apds9930_fops,
};
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	int err;
	u8 buf;
	APS_FUN();    
	struct apds9930_priv *obj = i2c_get_clientdata(client);  

	if(msg.event == PM_EVENT_SUSPEND) {   
		if(!obj) {
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}

		atomic_set(&obj->als_suspend, SUSPEND_ON);
              if(test_bit(CMC_BIT_ALS, &obj->enable)) {
                 if(err = apds9930_enable_als(client, false)) {
                    APS_ERR("disable als fail: %d\n", err);
                   }
                }
#ifndef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
		atomic_set(&obj->ps_suspend, SUSPEND_ON);
              if(test_bit(CMC_BIT_PS, &obj->enable)) {
                 if(err = apds9930_enable_ps(client, false)) {
                    APS_ERR("disable ps fail: %d\n", err);
                   }
                }
#endif
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_resume(struct i2c_client *client)
{

    struct apds9930_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if(!obj) {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    atomic_set(&obj->als_suspend, 0);
    if(!(test_bit(CMC_BIT_ALS, &obj->enable))) {
        if(err = apds9930_enable_als(client, true)) {
            APS_ERR("enable als fail: %d\n", err);
        }
    }

#ifndef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    atomic_set(&obj->ps_suspend, 0);
    if(!(test_bit(CMC_BIT_PS, &obj->enable))) {
        if(err = apds9930_enable_ps(client, true)) {
            APS_ERR("enable ps fail: %d\n", err);
        }
    }
#endif

    return 0;
}
/*----------------------------------------------------------------------------*/
static void apds9930_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
    APS_FUN();    
    struct apds9930_priv *obj = container_of(h, struct apds9930_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj) {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
    if(test_bit(CMC_BIT_ALS, &obj->enable)) {
        if(err = apds9930_enable_als(obj->client, false)) {
            APS_ERR("disable als fail: %d\n", err);
         }
    }
}
/*----------------------------------------------------------------------------*/
static void apds9930_late_resume(struct early_suspend *h)
{   /*late rusume is only applied for ALS*/
    APS_FUN();  
    struct apds9930_priv *obj = container_of(h, struct apds9930_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj) {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    if(!(test_bit(CMC_BIT_ALS, &obj->enable))) {
        if(err = apds9930_enable_als(obj->client, true)) {
            APS_ERR("enable als fail: %d\n", err);
         }
    }
  
}

int apds9930_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct apds9930_priv *obj = (struct apds9930_priv *)self;
	
	APS_FUN(f);
	switch (command) {
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {				
				value = *(int *)buff_in;
				if(value) {
					if(err = apds9930_enable_ps(obj->client, 1)) {
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				} else {
					if(err = apds9930_enable_ps(obj->client, 0)) {
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (hwm_sensor_data *)buff_out;	
				apds9930_read_ps(obj->client, &obj->ps);
				sensor_data->values[0] = apds9930_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int apds9930_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct apds9930_priv *obj = (struct apds9930_priv *)self;

	switch (command) {
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {
				value = *(int *)buff_in;				
				if(value) {
					if(err = apds9930_enable_als(obj->client, 1)) {
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				} else {
					if(err = apds9930_enable_als(obj->client, 0)) {
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (hwm_sensor_data *)buff_out;
				apds9930_read_als(obj->client, &obj->als);
				if(obj->als == 0) {
					sensor_data->values[0] = 1;				
				} else {
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++) {
					apds9930_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = apds9930_get_als_value(obj, obj->als);
				}
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int apds9930_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9930_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct apds9930_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	apds9930_obj = obj;

	obj->hw = get_cust_alsps_hw_apds9930();
	apds9930_get_addr(obj->hw, &obj->addr);
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
	INIT_WORK(&obj->eint_work, apds9930_eint_work);
#endif
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, SUSPEND_OFF);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
	
	apds9930_i2c_client = client;

	if(err = apds9930_init_client(client)) {
		goto exit_init_failed;
	}
	APS_LOG("apds9930_init_client() OK!\n");

	if(err = misc_register(&apds9930_device)) {
		APS_ERR("apds9930_device register failed\n");
		goto exit_misc_device_register_failed;
	}
#ifdef GN_MTK_AUTO_DETECT_ALSPS
      if(err = apds9930_create_attr(&(apds9930_init_info.platform_diver_addr->driver))) {
             APS_ERR("create attribute err = %d\n", err);
             goto exit_create_attr_failed;
       }
#else
      if(err = apds9930_create_attr(&apds9930_psensor_driver.driver)) {
             APS_ERR("create attribute err = %d\n", err);
             goto exit_create_attr_failed;
       }
#endif
	obj_ps.self = apds9930_obj;
	if(1 == obj->hw->polling_mode_ps) {
		obj_ps.polling = 1;
	} else {
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = apds9930_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps)) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = apds9930_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = apds9930_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als)) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = apds9930_early_suspend,
	obj->early_drv.resume   = apds9930_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	
       apds9930_init_flag = 0;
       return 0;

exit_create_attr_failed:
	misc_deregister(&apds9930_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
exit_kfree:
	kfree(obj);
exit:
	apds9930_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	apds9930_init_flag = -1;
    return err;
}
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_remove(struct i2c_client *client)
{
	int err;	
#ifdef GN_MTK_AUTO_DETECT_ALSPS
       if(err = apds9930_delete_attr(&(apds9930_init_info.platform_diver_addr->driver))) {
              APS_ERR("apds9930_delete_attr fail: %d\n", err);
       }
#else
       if(err = apds9930_delete_attr(&apds9930_psensor_driver.driver)) {
              APS_ERR("apds9930_delete_attr fail: %d\n", err);
    }
#endif
	if(err = misc_deregister(&(apds9930_init_info.platform_diver_addr->driver))) {
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	apds9930_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_AUTO_DETECT_ALSPS
static int apds9930_local_init(void)
{
    struct acc_hw *hw = get_cust_alsps_hw_apds9930();
    
    apds9930_power(hw, POWER_ON);
    if(i2c_add_driver(&apds9930_i2c_driver)) {
        APS_ERR("add driver error\n");
        return -1;
     }
    if(-1 == apds9930_init_flag) {
        return -1;
     }
    
    return 0;
}
#else
static int apds9930_probe(struct platform_device *pdev)
{
    struct alsps_hw *hw = get_cust_alsps_hwapds9930();      
    apds9930_power(hw, POWER_ON);
    if (i2c_add_driver(&apds9930_i2c_driver)) {
        APS_ERR("i2c_add_driver add driver error %d\n",__LINE__);
        return -1;
    }
    return 0;
}
#endif

static int apds9930_remove(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw_apds9930();
	APS_FUN();    
	apds9930_power(hw, POWER_OFF);    
	i2c_del_driver(&apds9930_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#ifndef GN_MTK_AUTO_DETECT_ALSPS
static struct platform_driver apds9930_psensor_driver = {
    .probe	= apds9930_probe,
    .remove	= apds9930_remove,
    .driver	= {
        .name	= "als_ps",
    }
};
#endif

static int __init apds9930_init(void)
{
	APS_FUN();
	i2c_register_board_info(I2C_NUM, &i2c_APDS9930, 1);
#ifdef GN_MTK_AUTO_DETECT_ALSPS
//    if(hwmsen_psensor_add(&apds9930_init_info)) {
//        APS_ERR("apds9930_init failed to register driver\n");
//        return -ENODEV;
//    }
#else 
    platform_driver_register(&apds9930_psensor_driver) {
        APS_ERR("apds9930_init failed to register driver\n");
        return -ENODEV;
    }
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit apds9930_exit(void)
{
#ifdef GN_MTK_AUTO_DETECT_ALSPS
//    hwmsen_psensor_del(&apds9930_init_info);
#else
    platform_driver_unregister(&apds9930_psensor_driver);              
#endif

}
/*----------------------------------------------------------------------------*/
module_init(apds9930_init);
module_exit(apds9930_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liya Lu");
MODULE_DESCRIPTION("apds9930 driver");
MODULE_LICENSE("GPL");
