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

/* drivers/hwmon/mt6516/amit/ltr558.c - LTR558 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

/*
 * Change log
 *
 * Date		Author		Description
 * ----------------------------------------------------------------------------
 * 11/17/2011	chenqy		Initial modification from ltr502.
 * 01/03/2012	chenqy		Fix logical error in sensor enable function.
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
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps_ltr558.h>
#include "gn_ltr558.h"
#include <linux/hwmsen_helper.h>

#include <linux/wakelock.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_reg_base.h>

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
        kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
        kal_bool auto_umask);

#define POWER_NONE_MACRO MT65XX_POWER_NONE
/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR558_I2C_ADDR_RAR	0 /*!< the index in obj->hw->i2c_addr: alert response address */
#define LTR558_I2C_ADDR_ALS	1 /*!< the index in obj->hw->i2c_addr: ALS address */
#define LTR558_I2C_ADDR_PS	2 /*!< the index in obj->hw->i2c_addr: PS address */
#define LTR558_DEV_NAME		"LTR558"
/*----------------------------------------------------------------------------*/
#define APS_TAG	"[ALS/PS] "
#define APS_FUN(f)      printk(KERN_ERR APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DEBUG
#if defined(APS_DEBUG)
#define APS_LOG(fmt, args...)	printk(KERN_ERR APS_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_ERR APS_TAG fmt, ##args)
#else
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

/******************************************************************************
 * extern functions
 *******************************************************************************/
/*----------------------------------------------------------------------------*/
#define mt6516_I2C_DATA_PORT		((base) + 0x0000)
#define mt6516_I2C_SLAVE_ADDR		((base) + 0x0004)
#define mt6516_I2C_INTR_MASK		((base) + 0x0008)
#define mt6516_I2C_INTR_STAT		((base) + 0x000c)
#define mt6516_I2C_CONTROL		((base) + 0x0010)
#define mt6516_I2C_TRANSFER_LEN		((base) + 0x0014)
#define mt6516_I2C_TRANSAC_LEN		((base) + 0x0018)
#define mt6516_I2C_DELAY_LEN		((base) + 0x001c)
#define mt6516_I2C_TIMING		((base) + 0x0020)
#define mt6516_I2C_START		((base) + 0x0024)
#define mt6516_I2C_FIFO_STAT		((base) + 0x0030)
#define mt6516_I2C_FIFO_THRESH		((base) + 0x0034)
#define mt6516_I2C_FIFO_ADDR_CLR	((base) + 0x0038)
#define mt6516_I2C_IO_CONFIG		((base) + 0x0040)
#define mt6516_I2C_DEBUG		((base) + 0x0044)
#define mt6516_I2C_HS			((base) + 0x0048)
#define mt6516_I2C_DEBUGSTAT		((base) + 0x0064)
#define mt6516_I2C_DEBUGCTRL		((base) + 0x0068)

/*----------------------------------------------------------------------------*/
static struct i2c_client *ltr558_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr558_i2c_id[] = {{LTR558_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ltr558={ I2C_BOARD_INFO(LTR558_DEV_NAME, LTR558_I2C_SLAVE_ADDR)};

/*----------------------------------------------------------------------------*/
static int ltr558_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltr558_i2c_remove(struct i2c_client *client);
static int ltr558_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr558_i2c_resume(struct i2c_client *client);
static int ltr558_enable_ps(struct i2c_client *client, bool enable);  

static struct ltr558_priv *g_ltr558_ptr = NULL;
/*----------------------------------------------------------------------------*/
//Gionee yanggy 2012-07-23 add for psensor compability begin
static int ltr558_local_init(void);
static int ltr558_remove(void);
static int ltr558_init_flag = 0;

static struct sensor_init_info ltr558_init_info = {
    .name = "psensor_ltr558",
    .init = ltr558_local_init,
    .uninit = ltr558_remove,
};
//Gionee yanggy 2012-07-23 add for psensor compability end
//Gionee mali modify for CR00764022 2013-1-15 begin
#if 0
struct lux_range {
    int range_lo;
    int range_hi;
};
static struct lux_range lux_level[10] = {
    {1,5},  //{5,7},   90
    {4,8},//{11,14}, 160
    {7,15},//{19,21}, 320
    {12,55},//{25,27}, 640
    {50,70},//{33,42}, 1280
    {65,120},//{50,120}, 2600
    {90,220},
    {180,1800},
    {1400,12000},
    {8000,65535}
};
static int lux_value[10] =  {1, 60,   90,  225, 640, 1280,  2600, 6400, 60000,    100000};
#endif
//Gionee mali modify for CR00764022 2013-1-15 end
//Gionee mali add calibration for ltr558 2012-9-28 begin 
#if defined(GN_PS_NEED_CALIBRATION)
struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int  valid;
};

static struct PS_CALI_DATA_STRUCT ps_cali={{0,0,0},};
#endif
//Gionee mali add calibration for ltr558 9-28 end

typedef enum {
    CMC_TRC_APS_DATA	= 0x0002,
    CMC_TRC_EINT		= 0x0004,
    CMC_TRC_IOCTL		= 0x0008,
    CMC_TRC_I2C		= 0x0010,
    CMC_TRC_CVT_ALS		= 0x0020,
    CMC_TRC_CVT_PS		= 0x0040,
    CMC_TRC_DEBUG		= 0x8000,
} CMC_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS		= 1,
    CMC_BIT_PS		= 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct ltr558_priv {
    struct alsps_hw *hw;
    struct i2c_client *client;
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    struct delayed_work eint_work;
#endif
    /*misc*/
    atomic_t	trace;
    atomic_t	i2c_retry;
    atomic_t	als_suspend;
    atomic_t	als_debounce;	/*debounce time after enabling als*/
    atomic_t	als_deb_on;	/*indicates if the debounce is on*/
    atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
    atomic_t	ps_mask;	/*mask ps: always return far away*/
    atomic_t	ps_debounce;	/*debounce time after enabling ps*/
    atomic_t	ps_deb_on;	/*indicates if the debounce is on*/
    atomic_t	ps_deb_end;	/*the jiffies representing the end of debounce*/
    atomic_t	ps_suspend;

    int		als;
    int		ps;
    u8		_align;
    u16		als_level_num;
    u16		als_value_num;
    u32		als_level[C_CUST_ALS_LEVEL-1];
    u32		als_value[C_CUST_ALS_LEVEL];

    bool		als_enable;	/*record current als status*/
    unsigned int	als_widow_loss;

    bool		ps_enable;	 /*record current ps status*/
    unsigned int	ps_thd_val;	 /*the cmd value can't be read, stored in ram*/
    ulong		enable;		 /*record HAL enalbe status*/
    ulong		pending_intr;	/*pending interrupt*/
    unsigned int	polling;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend	early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr558_i2c_driver = {
    .probe		= ltr558_i2c_probe,
    .remove		= ltr558_i2c_remove,
    .detect		= ltr558_i2c_detect,
    .suspend	= ltr558_i2c_suspend,
    .resume		= ltr558_i2c_resume,
    .id_table	= ltr558_i2c_id,
    .driver = {
        .owner	= THIS_MODULE,
        .name	= LTR558_DEV_NAME,
    },
};

static struct ltr558_priv *ltr558_obj = NULL;
static struct platform_driver ltr558_alsps_driver;

static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/*
 * The ps_trigger_xxx_table
 * controls the interrupt trigger range
 * bigger value means close to p-sensor
 * smaller value means far away from p-sensor
 */
#if 0
//Gionee: mali 2012-06-14 modify the CR00623845 from the old data to the new data for mmi test begin
static int ps_trigger_high_table[4] = {280, 280, 280, 280};
static int ps_trigger_low_table[4] = {240, 240, 240, 240};
static int ps_trigger_high = 380;//60;
static int ps_trigger_low = 250;//30;
//Gionee: mali 2012-06-14  modify the CR00623845 from the old data to the new data for mmi test end
#endif
static int ltr558_get_ps_value(struct ltr558_priv *obj, int ps);
static int ltr558_get_als_value(struct ltr558_priv *obj, int als);

/*----------------------------------------------------------------------------*/
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

static void ltr558_dumpReg(struct i2c_client *client)
{
    int i=0;
    u8 addr = 0x00;
    u8 regdata=0;
    for (i = 0; i < 9; i++) {
        // dump all
        hwmsen_read_byte_sr(client,addr,&regdata);
        APS_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        addr++;
        if (addr == 0x06)
            addr=addr+0x02;
        if (addr > 0x08)
            break;
    }
}

/*----------------------------------------------------------------------------*/
int ltr558_get_timing(void)
{
    return 200;
    /*
       u32 base = I2C2_BASE;
       return (__raw_readw(mt6516_I2C_HS) << 16) | (__raw_readw(mt6516_I2C_TIMING));
     */
}
/*----------------------------------------------------------------------------*/
int ltr558_config_timing(int sample_div, int step_div)
{
    u32 base = I2C2_BASE;
    unsigned long tmp;

    tmp = __raw_readw(mt6516_I2C_TIMING) & ~((0x7 << 8) | (0x1f << 0));
    tmp = (sample_div & 0x7) << 8 | (step_div & 0x1f) << 0 | tmp;

    return (__raw_readw(mt6516_I2C_HS) << 16) | (tmp);
}
/*----------------------------------------------------------------------------*/
int ltr558_read_data_als(struct i2c_client *client, int *data)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int ret = 0;
    int alsval_ch1_lo = 0;
    int alsval_ch1_hi = 0;
    int alsval_ch0_lo = 0;
    int alsval_ch0_hi = 0;
    int luxdata_int;
    int luxdata_flt;
    int ratio;
    int alsval_ch0;
    int alsval_ch1;

    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_0, &alsval_ch1_lo)) {
        APS_ERR("reads als data (ch1 lo) = %d\n", alsval_ch1_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_1, &alsval_ch1_hi)) {
        APS_ERR("reads aps data (ch1 hi) = %d\n", alsval_ch1_hi);
        return -EFAULT;
    }
    alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
    APS_LOG("alsval_ch1_hi=%x alsval_ch1_lo=%x\n",alsval_ch1_hi,alsval_ch1_lo);


    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_0, &alsval_ch0_lo)) {
        APS_ERR("reads als data (ch0 lo) = %d\n", alsval_ch0_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_1, &alsval_ch0_hi)) {
        APS_ERR("reads als data (ch0 hi) = %d\n", alsval_ch0_hi);
        return -EFAULT;
    }
    alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
    APS_LOG("alsval_ch0_hi=%x alsval_ch0_lo=%x\n",alsval_ch0_hi,alsval_ch0_lo);

    // FIXME:
    // make sure we will not meet div0 error
    if ((alsval_ch0 + alsval_ch1) == 0) {
        APS_ERR("Both CH0 and CH1 are zero\n");
        ratio = 1000;
    } else {
        ratio = (alsval_ch1 * 1000) / (alsval_ch0 + alsval_ch1);
    }

    // Compute Lux data from ALS data (ch0 and ch1)
    // Lux = (CH0*CH0_coeff - CH1*CH1_coeff)
    // Ratio = CH1 / (CH0 + CH1)
    //
    // FIXME: the below calibration does not match with pdf
    // For Ratio < 0.69:
    // 1.3618*CH0 - 1.5*CH1
    // For 0.69 <= Ratio < 1:
    // 0.57*CH0 - 0.345*CH1
    // For high gain, divide the calculated lux by 150.
#if 0
    if (ratio < 690){
        luxdata_flt = (13618 * alsval_ch0) - (15000 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else if ((ratio >= 690) && (ratio < 1000)){
        luxdata_flt = (5700 * alsval_ch0) - (3450 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else {
        luxdata_flt = 0;
    }
#else
    if (ratio < 450){
        luxdata_flt = (17743 * alsval_ch0) + (11059 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else if ((ratio >= 450) && (ratio < 640)){
        luxdata_flt = (37725 * alsval_ch0) - (13363 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else if ((ratio >= 640) && (ratio < 1000)){
        luxdata_flt = (16900 * alsval_ch0) - (1690 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else {
        luxdata_flt = 0;
    }
#endif
    printk("read_data_als ratio = %d \n",ratio);
    // For Range1
    if (als_gainrange == ALS_RANGE1_320)
        luxdata_flt = luxdata_flt / 150;
    //Gionee: mali 2012-06-14 modify the CR00623845 from the old data to the new data for mmi test begin
    
    if(luxdata_flt > 65535)
    {
        luxdata_flt = 65534;
    }
    if(luxdata_flt <0)
    {
        luxdata_flt = 65534;
    }
    
    //Gionee: mali 2012-06-14 modify the CR00623845 from the old data to the new data for mmi test end
    // convert float to integer;
    luxdata_int = luxdata_flt;
    if ((luxdata_flt - luxdata_int) > 0.5){
        luxdata_int = luxdata_int + 1;
    } else {
        luxdata_int = luxdata_flt;
    }

    if (atomic_read(&obj->trace) & CMC_TRC_APS_DATA) {
        APS_DBG("ALS (CH0): 0x%04X\n", alsval_ch0);
        APS_DBG("ALS (CH1): 0x%04X\n", alsval_ch1);
        APS_DBG("ALS (Ratio): %d\n", ratio);
        APS_DBG("ALS: %d\n", luxdata_int);
    }

    *data = luxdata_int;

    APS_LOG("luxdata_int=%x \n",luxdata_int);
    final_lux_val = luxdata_int;

    return 0;
}

int ltr558_read_data_ps(struct i2c_client *client, int *data)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int ret = 0;
    int psval_lo = 0;
    int psval_hi = 0;
    int psdata = 0;
    APS_FUN();
    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_0, &psval_lo)) {
        APS_ERR("reads aps data = %d\n", psval_lo);
        return -EFAULT;
    }
    printk("ltr558_read_data_ps psval_lo = %x \n",psval_lo);

    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_1, &psval_hi)) {
        APS_ERR("reads aps hi data = %d\n", psval_hi);
        return -EFAULT;
    }
    
    printk("ltr558_read_data_ps psval_hi = %x \n",psval_hi);
    psdata = ((psval_hi & 7) * 256) + psval_lo;

    *data = psdata;
    
    printk("ltr558_read_data_ps psdata = %x \n",psdata);


    return 0;

}
//#if 0
static int ltr558_ps_enable(struct i2c_client *client, int gainrange)
{
    int error;
    int setgain;

    switch (gainrange) {
        case PS_RANGE1:
            setgain = MODE_PS_ON_Gain1;
            //ps_trigger_high = ps_trigger_high_table[0];
            //ps_trigger_low = ps_trigger_low_table[0];
            break;

        case PS_RANGE2:
            setgain = MODE_PS_ON_Gain4;
            //ps_trigger_high = ps_trigger_high_table[1];
            //ps_trigger_low = ps_trigger_low_table[1];
            break;

        case PS_RANGE4:
            setgain = MODE_PS_ON_Gain8;
            //ps_trigger_high = ps_trigger_high_table[2];
            //ps_trigger_low = ps_trigger_low_table[2];
            break;

        case PS_RANGE8:
            setgain = MODE_PS_ON_Gain16;
            //ps_trigger_high = ps_trigger_high_table[3];
            //ps_trigger_low = ps_trigger_low_table[3];
            break;

        default:
            setgain = MODE_PS_ON_Gain1;
            //ps_trigger_high = ps_trigger_high_table[0];
            //ps_trigger_low = ps_trigger_low_table[0];
            break;
    }

    error = hwmsen_write_byte(client, APS_RW_PS_CONTR, setgain);
    mdelay(WAKEUP_DELAY);

    /* ===============
     * ** IMPORTANT **
     * ===============
     * Other settings like timing and threshold to be set here, if required.
     * Not set and kept as device default for now.
     */

    return error;
}

static int ltr558_als_enable(struct i2c_client *client, int gainrange)
{
    int error;

    if (gainrange == 1) {
        error = hwmsen_write_byte(client, APS_RW_ALS_CONTR, MODE_ALS_ON_Range1);
    } else if (gainrange == 2) {
        error = hwmsen_write_byte(client, APS_RW_ALS_CONTR, MODE_ALS_ON_Range2);
    } else {
        error = -1;
    }

    mdelay(WAKEUP_DELAY);

    /* ==============
     * ** IMPORTANT **
     * ===============
     * Other settings like timing and threshold to be set here, if required.
     * Not set and kept as device default for now.
     */

    return error;
}
//#endif

/*----------------------------------------------------------------------------*/

int ltr558_init_device(struct i2c_client *client)
{
    u8 buf =0;
    int i = 0;
    int ret = 0;
#if 0
    int init_ps_gain;
    int init_als_gain;
    int data;
#endif
    //Gionee yangy 2012-07-23 modify for optimization begin
    //Gionee yanggy 2012-07-21 add for ps_interrupt mode begin
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    if (hwmsen_write_byte(client, APS_RW_INTERRUPT, 0x09))      //interrupt mode >>Only PS measurement can trigger interrupt
    {
        APS_ERR("ltr558 write 0x8f error\n");
        return -EFAULT;
    }
    if(hwmsen_write_byte(client, APS_RW_INTERRUPT_PERSIST, 0x80))        //4 PS measurement data will generate an interrupt
    {
        APS_ERR("ltr558 write to 0x9E err!\n");
        return -EFAULT;
    }
#endif 
    if(hwmsen_write_byte(client,APS_RW_PS_N_PULSES,0x12))        //led pulse number 4 
    {    
        APS_ERR("ltr558 write to 0x83 err!\n");
        return -EFAULT;
    }    
    
    if(hwmsen_write_byte(client, APS_RW_PS_LED, 0xff))  //pulse modulation frequency led peak current
    {    
        APS_ERR("ltr558 write to 0x82 err!\n");
        return -EFAULT;
    }
    //Gionee yanggy 2012-07-21 add for ps_interrupt mode end
    if(hwmsen_write_byte(client, APS_RW_PS_THRES_UP_0, PS_THRES_UP_0_VALUE))      //high threshold
    {
        APS_ERR("ltr558 write to 0x90 err!\n");
        return -EFAULT;
    }

    if(hwmsen_write_byte(client, APS_RW_PS_THRES_UP_1, PS_THRES_UP_1_VALUE))
    {
        APS_ERR("ltr558 write to 0x91 err!\n");
        return -EFAULT;
    }

    if(hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_0, PS_THRES_LOW_0_VALUE))    //low theshold
    {
        APS_ERR("ltr558 write to 0x92 err!\n");
        return -EFAULT;
    }

    if(hwmsen_write_byte(client, APS_RW_PS_THRES_LOW_1, PS_THRES_LOW_1_VALUE))
    {
        APS_ERR("ltr558 write to 0x93 err!\n");
        return -EFAULT;
    }

    if(hwmsen_write_byte(client,APS_RW_PS_MEAS_RATE,0x00))        //ps measure rate >>50ms
    {
        APS_ERR("ltr558 write to 0x84 err!\n");
        return -EFAULT;
    }

    mdelay(WAKEUP_DELAY);
#if 0
    // Enable PS to Gain1 at startup
    init_ps_gain = PS_RANGE8;
    ps_gainrange = init_ps_gain;

    ret = ltr558_ps_enable(client, init_ps_gain);
    if (ret < 0)
    {
        APS_ERR("init ps fail !!!!\n",__LINE__);
        return -EFAULT;
    }

    // Enable ALS to Full Range at startup
    init_als_gain = ALS_RANGE2_64K;
    als_gainrange = init_als_gain;

    ret = ltr558_als_enable(client, init_als_gain);
    if (ret < 0)
    {
        APS_ERR("init als fail !!!!\n",__LINE__);
        return -EFAULT;
    }
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static void ltr558_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;

    APS_LOG("power %s\n", on ? "on" : "off");
    APS_LOG("power id:%d POWER_NONE_MACRO:%d\n", hw->power_id, POWER_NONE_MACRO);

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "LTR558"))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "LTR558"))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
}
/*----------------------------------------------------------------------------*/
static int ltr558_enable_als(struct i2c_client *client, bool enable)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err=0;
    u8 regdata=0;

    if(enable == obj->als_enable)
    {
        return 0;
    }

    if (hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata))
    {
        APS_ERR("read APS_RW_ALS_CONTR register err!\n");
        return -1;
    }

    if(enable == TRUE)//enable als
    {
        APS_LOG("ALS(1): enable als only \n");
        regdata |= 0b00000010;
    }
    else
    {
        APS_LOG("ALS(1): disable als only \n");
        regdata &= 0b00000000;	// de-active the als

    }
    if (hwmsen_write_byte(client, APS_RW_ALS_CONTR, regdata)) {
        APS_LOG("ltr558_enable_als failed!\n");
        return -1;
    }

    obj->als_enable = enable;

    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr558_enable_ps(struct i2c_client *client, bool enable)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    u8 regdata = 0;
    u8 regint = 0;
    int i;

    APS_LOG(" ltr558_enable_ps: enable:  %d, obj->ps_enable: %d\n",enable, obj->ps_enable);
    if(enable == obj->ps_enable)
    {
        return 0;
    }

    if(hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata))
    {
        APS_ERR("read APS_RW_PS_CONTR register err!\n");
        return -1;
    }

    if(enable == TRUE)//enable ps
    {
        regdata |= 0b00000010;
        APS_LOG("PS(0): enable +++++++++++\n");
    }
    else//disable ps
    {		
        APS_LOG("PS(0): disable -----------------\n");
        regdata &= 0b00000000;	// de-active the ps
    }

    APS_LOG(" write ltr558_enable_ps regdata:%x \n",regdata);
    if (hwmsen_write_byte(client, APS_RW_PS_CONTR, regdata)) {
        APS_ERR("ltr558_enable_ps failed!\n");
        return -1;
    }

    if (hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata)) {
        APS_ERR("ltr558_read failed!\n");
        return -1;
    }
    
    for (i = 0; i < 300; i++) {
        mdelay(WAKEUP_DELAY);
        hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata);
        if ((enable == true && (regdata & 0x02) == 0x02) || (enable == false && (regdata & 0x00) == 0x00)) {
            break;
        }
    }
    if ( i >= 300) {
        APS_ERR("ltr558_enable_ps read the APS_RW_PS_CONTR failed\n");
        return -1;
    }

    obj->ps_enable = enable;

#if 0
#if defined(GN_PS_NEED_CALIBRATION)
    if(ps_cali.valid ==1) 
    {
        hwmsen_write_byte(obj->client, 0x90, (ps_cali.close) & 0x00ff);
        hwmsen_write_byte(obj->client, 0x91, ((ps_cali.close)>>8) & 0x07);
        hwmsen_write_byte(obj->client, 0x92, (ps_cali.far_away)& 0x00ff);
        hwmsen_write_byte(obj->client, 0x93, ((ps_cali.far_away)>>8)&0x07);
    }
    else 
    {
        hwmsen_write_byte(obj->client, 0x90, PS_THRES_UP_0_VALUE);
        hwmsen_write_byte(obj->client, 0x91, PS_THRES_UP_1_VALUE);
        hwmsen_write_byte(obj->client, 0x92, PS_THRES_LOW_0_VALUE);
        hwmsen_write_byte(obj->client, 0x93, PS_THRES_LOW_1_VALUE);
    } 
#else
    hwmsen_write_byte(obj->client, 0x90, PS_THRES_UP_0_VALUE);
    hwmsen_write_byte(obj->client, 0x91, PS_THRES_UP_1_VALUE);
    hwmsen_write_byte(obj->client, 0x92, PS_THRES_LOW_0_VALUE);
    hwmsen_write_byte(obj->client, 0x93, PS_THRES_LOW_1_VALUE);
#endif
#endif
    return err;
}
/*----------------------------------------------------------------------------*/
//Gionee yanggy 2012-07-21 add for ps_intrrupt mode begin
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
static int ltr558_check_intr(struct i2c_client *client)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err;
    u8 data=0;

    err = hwmsen_read_byte_sr(client,APS_RO_ALS_PS_STATUS,&data);

    if (err) {
        APS_ERR("WARNING: read int status: %d\n", err);
        return -1;
    }

    if (data & 0x02) {
        set_bit(CMC_BIT_PS, &obj->pending_intr);
    } else {
        clear_bit(CMC_BIT_PS, &obj->pending_intr);
    }

    if (atomic_read(&obj->trace) & CMC_TRC_DEBUG) {
        APS_LOG("check intr: 0x%08X\n", obj->pending_intr);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
void ltr558_eint_func(void)
{
    struct ltr558_priv *obj = g_ltr558_ptr;
    if(!obj)
    {
        return;
    }

    schedule_delayed_work(&obj->eint_work,0);
    if(atomic_read(&obj->trace) & CMC_TRC_EINT)
    {
        APS_LOG("eint: als/ps intrs\n");
    }
}
/*----------------------------------------------------------------------------*/
static void ltr558_eint_work(struct work_struct *work)
{
    struct ltr558_priv *obj = (struct ltr558_priv *)container_of(work, struct ltr558_priv, eint_work);
    int err;
    u8 buf;
    hwm_sensor_data sensor_data;

    APS_FUN();
    memset(&sensor_data, 0, sizeof(sensor_data));

    err = ltr558_check_intr(obj->client);
    if (err) {
        APS_ERR("check intrs: %d\n", err);
        return -1;
    }

    if ((1 << CMC_BIT_PS) & obj->pending_intr) {
        
        if (err = ltr558_read_data_ps(obj->client, &obj->ps)) {
            APS_ERR("ltr558 read ps data: %d\n", err);;
        }
        //map and store data to hwm_sensor_data
        sensor_data.values[0] = ltr558_get_ps_value(obj, obj->ps);

        if(sensor_data.values[0] == 0)     //set the ps_threshold dynamiclly
        {
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,0xff))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }

            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,0x07))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
//Gionee mali add calibration for ltr558 2012-9-28 begin
#if defined(GN_PS_NEED_CALIBRATION)
            if(ps_cali.valid == 0)
            {
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,PS_THRES_LOW_0_VALUE))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
            
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,PS_THRES_LOW_1_VALUE))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
            }
            else if(ps_cali.valid == 1)
            {
                if(hwmsen_write_byte(obj->client, APS_RW_PS_THRES_LOW_0, (ps_cali.far_away) & 0x00ff))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
                
                if(hwmsen_write_byte(obj->client, APS_RW_PS_THRES_LOW_1, ((ps_cali.far_away)>>8) & 0x07))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
            }
#else
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,PS_THRES_LOW_0_VALUE))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
           
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,PS_THRES_LOW_1_VALUE))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
#endif
//Gionee mali add calibration for ltr558 2012-9-28 end
        }
        else if(sensor_data.values[0] == 1)    //set the ps_threshold dynamiclly
        {
//Gionee mali add calibration for ltr558 2012-9-28 begin
#if defined(GN_PS_NEED_CALIBRATION)
            if(ps_cali.valid == 0)
            {
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,PS_THRES_UP_0_VALUE))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
            
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,PS_THRES_UP_1_VALUE))
                {
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }
            }else if(ps_cali.valid == 1)
            {
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0, ps_cali.close & 0x00ff))
                {    
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                }    
                
                if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1, ((ps_cali.close)>> 8) & 0x07))
                {    
                    APS_ERR("%s: error\n", __func__);
                    return -EFAULT;
                } 
            }
#else
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,PS_THRES_UP_0_VALUE))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
            
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,PS_THRES_UP_1_VALUE))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
#endif
//Gionee mali add calibration for ltr558 2012-9-28 end
            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,0x00))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }

            if(hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,0x00))
            {
                APS_ERR("%s: error\n", __func__);
                return -EFAULT;
            }
        }
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        //let up layer to know
        APS_LOG("ltr558 read ps data = %d \n",sensor_data.values[0]);
        if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)) {
            APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
        }
    }


    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
}

/*----------------------------------------------------------------------------*/
int ltr558_setup_eint(struct i2c_client *client)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);

    APS_FUN();
    g_ltr558_ptr = obj;
    /*configure to GPIO function, external interrupt*/

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ltr558_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);	
    
    return 0;
}
#endif 
//Gionee yanggy 2012-07-21 add for ps_interrupt mode end
/*----------------------------------------------------------------------------*/
static int ltr558_init_client(struct i2c_client *client)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err=0;
    APS_FUN();	


    if((err = ltr558_init_device(client)))
    {
        APS_ERR("ltr558_init_device init dev: %d\n", err);
        return err;
    }
    //Gionee yanggy 2012-07-21 add for ps_interrupt mode begin
#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    if((err = ltr558_setup_eint(client)))
    {
        APS_ERR("setup eint: %d\n", err);
        return err;
    }
#endif
    //Gionee yanggy 2012-07-21 add for ps_interrupt mode end

    return err;
}
/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
static ssize_t ltr558_show_config(struct device_driver *ddri, char *buf)
{
    ssize_t res;

    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
            atomic_read(&ltr558_obj->i2c_retry), atomic_read(&ltr558_obj->als_debounce),
            atomic_read(&ltr558_obj->ps_mask), ltr558_obj->ps_thd_val, atomic_read(&ltr558_obj->ps_debounce));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_store_config(struct device_driver *ddri, char *buf, size_t count)
{
    int retry, als_deb, ps_deb, mask, thres;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
    {
        atomic_set(&ltr558_obj->i2c_retry, retry);
        atomic_set(&ltr558_obj->als_debounce, als_deb);
        atomic_set(&ltr558_obj->ps_mask, mask);
        ltr558_obj->ps_thd_val= thres;
        atomic_set(&ltr558_obj->ps_debounce, ps_deb);
    }
    else
    {
        APS_ERR("invalid content: '%s', length = %d\n", buf, count);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_trace(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr558_obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_store_trace(struct device_driver *ddri, char *buf, size_t count)
{
    int trace;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&ltr558_obj->trace, trace);
    }
    else
    {
        APS_ERR("invalid content: '%s', length = %d\n", buf, count);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_als(struct device_driver *ddri, char *buf)
{
    int res;
    u8 dat = 0;

    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }
    // if(res = ltr558_read_data(ltr558_obj->client, &ltr558_obj->als))
    if(res = ltr558_read_data_als(ltr558_obj->client, &ltr558_obj->als))
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        // dat = ltr558_obj->als & 0x3f;
        dat = ltr558_obj->als;
        return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_ps(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    u8 dat=0;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    if(res = ltr558_read_data_ps(ltr558_obj->client, &ltr558_obj->ps))
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {
        dat = ltr558_get_ps_value(ltr558_obj, ltr558_obj->ps);
        return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_ps_raw(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    int dat=0;
    int data = -1;
    int err = 0 ;
    
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }
    
    if(err = ltr558_enable_ps(ltr558_obj->client, true))
    {
        APS_ERR("enable ps fail: %d\n", err);
        return -1;
    }
    
    set_bit(CMC_BIT_PS, &ltr558_obj->enable);
    msleep(300);
    
    if(res = ltr558_read_data_ps(ltr558_obj->client, &ltr558_obj->ps))
    {
        snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else
    {   
        APS_LOG("weiwei dat = %d\n",dat);
        dat = ltr558_obj->ps & 0x80;
        data = dat & 0x0000FFFF;
        APS_LOG("weiwei temp1 = %d\n",data);
        dat = ltr558_get_ps_value(ltr558_obj, ltr558_obj->ps);
        data = ((dat<<16) & 0xFFFF0000) | data;
        APS_LOG("weiwei temp2 = %d\n",data);
    }
    
    msleep(50);
    if(err = ltr558_enable_ps(ltr558_obj->client, false))
    {
        APS_ERR("disable ps fail: %d\n", err);
        return -1;
    }
    clear_bit(CMC_BIT_PS, &ltr558_obj->enable);
    
    return snprintf(buf, PAGE_SIZE, "%08X\n", data);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_reg(struct device_driver *ddri, char *buf)
{
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    /*read*/
    ltr558_dumpReg(ltr558_obj->client);

    return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    if(ltr558_obj->hw)
    {

        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
                ltr558_obj->hw->i2c_num, ltr558_obj->hw->power_id, ltr558_obj->hw->power_vol);

    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }

#ifdef MT6516
    len += snprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
            CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

    len += snprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN,
            mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN),
            mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
#endif

    len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr558_obj->als_suspend), atomic_read(&ltr558_obj->ps_suspend));

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_i2c(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    u32 base = I2C2_BASE;

    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "DATA_PORT	= 0x%08X\n", __raw_readl(mt6516_I2C_DATA_PORT	));
    len += snprintf(buf+len, PAGE_SIZE-len, "SLAVE_ADDR	= 0x%08X\n", __raw_readl(mt6516_I2C_SLAVE_ADDR));
    len += snprintf(buf+len, PAGE_SIZE-len, "INTR_MASK	= 0x%08X\n", __raw_readl(mt6516_I2C_INTR_MASK));
    len += snprintf(buf+len, PAGE_SIZE-len, "INTR_STAT	= 0x%08X\n", __raw_readl(mt6516_I2C_INTR_STAT));
    len += snprintf(buf+len, PAGE_SIZE-len, "CONTROL	= 0x%08X\n", __raw_readl(mt6516_I2C_CONTROL));
    len += snprintf(buf+len, PAGE_SIZE-len, "TRANSFER_LEN	= 0x%08X\n", __raw_readl(mt6516_I2C_TRANSFER_LEN));
    len += snprintf(buf+len, PAGE_SIZE-len, "TRANSAC_LEN	= 0x%08X\n", __raw_readl(mt6516_I2C_TRANSAC_LEN));
    len += snprintf(buf+len, PAGE_SIZE-len, "DELAY_LEN	= 0x%08X\n", __raw_readl(mt6516_I2C_DELAY_LEN));
    len += snprintf(buf+len, PAGE_SIZE-len, "TIMING		= 0x%08X\n", __raw_readl(mt6516_I2C_TIMING));
    len += snprintf(buf+len, PAGE_SIZE-len, "START		= 0x%08X\n", __raw_readl(mt6516_I2C_START));
    len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_STAT	= 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_STAT));
    len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_THRESH	= 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_THRESH));
    len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_ADDR_CLR	= 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_ADDR_CLR));
    len += snprintf(buf+len, PAGE_SIZE-len, "IO_CONFIG	= 0x%08X\n", __raw_readl(mt6516_I2C_IO_CONFIG));
    len += snprintf(buf+len, PAGE_SIZE-len, "DEBUG		= 0x%08X\n", __raw_readl(mt6516_I2C_DEBUG));
    len += snprintf(buf+len, PAGE_SIZE-len, "HS		= 0x%08X\n", __raw_readl(mt6516_I2C_HS));
    len += snprintf(buf+len, PAGE_SIZE-len, "DEBUGSTAT	= 0x%08X\n", __raw_readl(mt6516_I2C_DEBUGSTAT));
    len += snprintf(buf+len, PAGE_SIZE-len, "DEBUGCTRL	= 0x%08X\n", __raw_readl(mt6516_I2C_DEBUGCTRL));

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_store_i2c(struct device_driver *ddri, char *buf, size_t count)
{
    int sample_div, step_div;
    unsigned long tmp;
    u32 base = I2C2_BASE;

    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }
    else if(2 != sscanf(buf, "%d %d", &sample_div, &step_div))
    {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }
    tmp = __raw_readw(mt6516_I2C_TIMING) & ~((0x7 << 8) | (0x1f << 0));
    tmp = (sample_div & 0x7) << 8 | (step_div & 0x1f) << 0 | tmp;
    __raw_writew(tmp, mt6516_I2C_TIMING);

    return count;
}
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr558_priv *obj, const char* buf, size_t count,
        u32 data[], int len)
{
    int idx = 0;
    char *cur = (char*)buf, *end = (char*)(buf+count);

    while(idx < len)
    {
        while((cur < end) && IS_SPACE(*cur))
        {
            cur++;
        }

        if(1 != sscanf(cur, "%d", &data[idx]))
        {
            break;
        }

        idx++;
        while((cur < end) && !IS_SPACE(*cur))
        {
            cur++;
        }
    }
    return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_alslv(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr558_obj->als_level_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr558_obj->hw->als_level[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_store_alslv(struct device_driver *ddri, char *buf, size_t count)
{
    struct ltr558_priv *obj;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr558_obj->als_level, ltr558_obj->hw->als_level, sizeof(ltr558_obj->als_level));
    }
    else if(ltr558_obj->als_level_num != read_int_from_buf(ltr558_obj, buf, count,
                ltr558_obj->hw->als_level, ltr558_obj->als_level_num))
    {
        APS_ERR("invalid format: '%s'\n", buf);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_show_alsval(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr558_obj->als_value_num; idx++)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr558_obj->hw->als_value[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr558_store_alsval(struct device_driver *ddri, char *buf, size_t count)
{
    if(!ltr558_obj)
    {
        APS_ERR("ltr558_obj is null!!\n");
        return 0;
    }
    else if(!strcmp(buf, "def"))
    {
        memcpy(ltr558_obj->als_value, ltr558_obj->hw->als_value, sizeof(ltr558_obj->als_value));
    }
    else if(ltr558_obj->als_value_num != read_int_from_buf(ltr558_obj, buf, count,
                ltr558_obj->hw->als_value, ltr558_obj->als_value_num))
    {
        APS_ERR("invalid format: '%s'\n", buf);
    }
    return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,	 S_IWUSR | S_IRUGO, ltr558_show_als,	NULL);
static DRIVER_ATTR(ps,	 S_IWUSR | S_IRUGO, ltr558_show_ps,	NULL);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, ltr558_show_config,ltr558_store_config);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, ltr558_show_alslv, ltr558_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ltr558_show_alsval,ltr558_store_alsval);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, ltr558_show_trace, ltr558_store_trace);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ltr558_show_status, NULL);
static DRIVER_ATTR(reg,	 S_IWUSR | S_IRUGO, ltr558_show_reg, NULL);
static DRIVER_ATTR(i2c,	 S_IWUSR | S_IRUGO, ltr558_show_i2c, ltr558_store_i2c);
//static DRIVER_ATTR(adjust, S_IWUSR | S_IRUGO, ltr558_ps_adjust, NULL);
static DRIVER_ATTR(alsps,  S_IWUSR | S_IRUGO, ltr558_show_ps_raw, NULL);

/*----------------------------------------------------------------------------*/
static struct device_attribute *ltr558_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,
    &driver_attr_trace,		/*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_i2c,
    &driver_attr_reg,
    &driver_attr_alsps
};
/*----------------------------------------------------------------------------*/
static int ltr558_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(ltr558_attr_list)/sizeof(ltr558_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, ltr558_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", ltr558_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr558_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(ltr558_attr_list)/sizeof(ltr558_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, ltr558_attr_list[idx]);
    }

    return err;
}
/******************************************************************************
 * Function Configuration
 ******************************************************************************/

static int ltr558_get_als_value(struct ltr558_priv *obj, int als)
{
    int idx;
    int invalid = 0;
    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(als <= obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }

    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        }
        return obj->hw->als_value[idx];
    }
    else
    {
        if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        }
        return -1;
    }
}

#if 0
//Gionee mali modify for CR00764022 2013-1-15 begin
static int ltr558_get_als_value(struct ltr558_priv *obj, int als)
{
    int idx,i;
    int invalid = 0;
    int tmp_level= 0;
    int tmp_idx[2] = {0xff,0xff};
    static int last_lux_level = 0;

    APS_DBG("\nliujiang: input als value = %d,  last_idx = %d\n",als,last_lux_level);
    for(idx = 0;idx < 10; idx++)
    {
        if(als <=5)
        {
             return lux_value[0];
        }
        else if( (als >= lux_level[idx].range_lo)&&(als <= lux_level[idx].range_hi) )
        {
            tmp_idx[0] = idx;

            if( (als >= lux_level[idx+1].range_lo)&&(als <= lux_level[idx+1].range_hi) )
            {
                tmp_idx[1] = idx + 1;

            }
	    else
           {
              tmp_idx[1] = idx; 
           }
            break;
        }
    }
    
    APS_DBG("liujiang: idx_0 = %d,   idx_1 = %d\n",tmp_idx[0],tmp_idx[1]);
    if((0xff == tmp_idx[0])&&(0xff == tmp_idx[1]))
    {
        APS_ERR("lux value out of range\n");
        return -1;
    }
    else if((0xff != tmp_idx[0])&&(0xff != tmp_idx[1]))
    {
        int change_value_0,change_value_1;

        change_value_0 = (tmp_idx[0] >= last_lux_level) ? (tmp_idx[0] - last_lux_level) : (last_lux_level - tmp_idx[0]);
        change_value_1 = (tmp_idx[1] >= last_lux_level) ? (tmp_idx[1] - last_lux_level) : (last_lux_level - tmp_idx[1]);
        tmp_level = (change_value_0 <= change_value_1) ? tmp_idx[1] : tmp_idx[0];
    }
    else
    {
        tmp_level = tmp_idx[0];
    }

LEVEL_0:
    APS_ERR("liujiang: alculate idx = %d\n",tmp_level);
    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_DBG("ALS: %05d => %05d \n", als, lux_value[tmp_level]);
        }
        last_lux_level = tmp_level;
        return lux_value[tmp_level];
    }
    else
    {
        if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
        {
            APS_DBG("ALS: %05d => %05d (-1)\n", als, lux_value[tmp_level]);
        }
        return -1;
    }
}
//Gionee mali modify for CR00764022 2013-1-15 end
#endif
/*----------------------------------------------------------------------------*/

static int ltr558_get_ps_value(struct ltr558_priv *obj, int ps)
{
    int val= -1;
    static int val_temp = 1;

    //Gionee mali add calibration for ltr558 2012-9-28 begin
#if defined(GN_PS_NEED_CALIBRATION)
    APS_LOG(" ps=%x,ps_cali.far_away = %x , ps_cali.close = %x ,ps_valid = %x \n", ps, ps_cali.far_away, ps_cali. close, ps_cali.valid);
    if(ps_cali.valid == 1)
   {   
        if(ps > ps_cali.close)
        {
            val = 0;
            val_temp = 0;
        }
        else if(ps < ps_cali.far_away)
        {
            val = 1;
            val_temp = 1;
        }
        else
        {
            val = val_temp;
        }

   }else
   {
       if(ps >= PS_THRES_UP_VALUE)
       {
           val = 0;
           val_temp = 0;
       }else if(ps <= PS_THRES_LOW_VALUE)
       {
           val = 1;
           val_temp = 1;
       }
       else
       {
           val = val_temp;
       }
//Gionee mali modify for interrupt mode 2012-8-7 end
   }
#else
   if(ps>=PS_THRES_UP_VALUE)                                                                             
   {
       val = 0;
       val_temp = 0;
   }
   else if(ps <= PS_THRES_LOW_VALUE)
   {
       val = 1;
       val_temp = 1;
   }
   else
   {
       val = val_temp;
   }
#endif
//Gionee mali add calibration for ltr558 2012-9-28 end
   return val;
}


//Gionee mali add calibration for ltr558 2012-9-28 begin
#if defined(GN_PS_NEED_CALIBRATION)
static ltr558_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{
   if(data_cali->valid ==1)
   {
       ps_cali.close = data_cali->close; 
       ps_cali.far_away = data_cali->far_away;
       ps_cali.valid = 1;
       
       hwmsen_write_byte(ltr558_obj->client, 0x90, (ps_cali.close) & 0x00ff);
       hwmsen_write_byte(ltr558_obj->client, 0x91, ((ps_cali.close)>>8) & 0x07);
       hwmsen_write_byte(ltr558_obj->client, 0x92, (ps_cali.far_away)& 0x00ff);
       hwmsen_write_byte(ltr558_obj->client, 0x93, ((ps_cali.far_away)>>8)&0x07);
   }
   else
   {
       ps_cali.valid = 0; 
       hwmsen_write_byte(ltr558_obj->client, 0x90, PS_THRES_UP_0_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x91, PS_THRES_UP_1_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x92, PS_THRES_LOW_0_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x93, PS_THRES_LOW_1_VALUE);
    }
}

static int ltr558_read_data_for_cali(struct i2c_client *client,struct PS_CALI_DATA_STRUCT *ps_data_cali)
{
    
    int ret=0;
    int i=0;
    int data[COUNT];
    int data_total=0;
    ssize_t len = 0;
    int noise = 0;
    int max = 0;
    int buf;
    int err = 0;
    int j = 0;
    hwm_sensor_data sensor_data;

    if(!ltr558_obj)
    { 
        APS_ERR("ltr558_obj is null!!\n");
        len = sprintf(buf, "ltr501_obj is null\n");
        goto report_value;
    }

    // wait for register to be stable
    for (i = 0; i < COUNT; i++) 
    {
        // wait for ps value be stable
        if (max++ > 50) {
            ps_cali.valid = 0;
            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;
            
            goto report_value;
        }
        
        mdelay(50);
        ret=ltr558_read_data_ps(ltr558_obj->client,&data[i]);

        APS_ERR("the register of APS_RW_PS_CONTR data[i] is %x \n", data[i]);

        if (ret < 0) {
            i--;
            continue;
        }

        data_total+= data[i];
        if(data[i] == 0)
        {
            j++;
        }
    }
    
    if(data_total == 0)
    {
        ps_data_cali->close = NOISE_HIGH;
        ps_data_cali->far_away = NOISE_LOW;
        ps_data_cali->valid = 1;

        ps_cali.close = NOISE_HIGH;
        ps_cali.far_away = NOISE_LOW;
        ps_cali.valid = 1;
    }
    else
    {
        noise=data_total/(COUNT-j);
        if(noise > NOISE_MAX) {
            ps_cali.far_away = 0;
            ps_cali.close = 0;
            ps_cali.valid = 0;

            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;
        }
        else 
        {
            ps_data_cali->close = noise + NOISE_HIGH; 
            ps_data_cali->far_away = noise + NOISE_LOW;
            ps_data_cali->valid = 1;

            ps_cali.close = noise + NOISE_HIGH;
            ps_cali.far_away = noise + NOISE_LOW;
            ps_cali.valid = 1;
        }
        
    }
    
   if(ps_cali.valid ==1)
   {
       hwmsen_write_byte(ltr558_obj->client, 0x90, (ps_cali.close) & 0x00ff);
       hwmsen_write_byte(ltr558_obj->client, 0x91, ((ps_cali.close)>>8) & 0x07);
       hwmsen_write_byte(ltr558_obj->client, 0x92, (ps_cali.far_away)& 0x00ff);
       hwmsen_write_byte(ltr558_obj->client, 0x93, ((ps_cali.far_away)>>8)&0x07);
   }
   else
   {
       hwmsen_write_byte(ltr558_obj->client, 0x90, PS_THRES_UP_0_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x91, PS_THRES_UP_1_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x92, PS_THRES_LOW_0_VALUE);
       hwmsen_write_byte(ltr558_obj->client, 0x93, PS_THRES_LOW_1_VALUE);
    }
    
report_value:
    sensor_data.values[0] = 1;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    if(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)) {
        APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
    }

    return 0;
}
#endif
//Gionee mali add calibration for ltr558 2012-9-28 end

/******************************************************************************
 * Function Configuration
 ******************************************************************************/
static int ltr558_open(struct inode *inode, struct file *file)
{
    file->private_data = ltr558_i2c_client;

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr558_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long ltr558_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
#if defined(GN_PS_NEED_CALIBRATION)
    struct PS_CALI_DATA_STRUCT ps_cali_temp;
#endif
    APS_LOG("ltr558 cmd = %d \n", cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if(err = ltr558_enable_ps(obj->client, true))
                {
                    APS_ERR("enable ps fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_PS, &obj->enable);
            }
            else
            {
                if(err = ltr558_enable_ps(obj->client, false))
                {
                    APS_ERR("disable ps fail: %d\n", err);
                    goto err_out;
                }
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            break;

        case ALSPS_GET_PS_MODE:
            enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
            if(err = ltr558_read_data_ps(obj->client, &obj->ps))
            {
                goto err_out;
            }
            dat = ltr558_get_ps_value(obj, obj->ps);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            if(err = ltr558_read_data_ps(obj->client, &obj->ps))
            {
                goto err_out;
            }

            dat = obj->ps & 0x80;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if(err = ltr558_enable_als(obj->client, true))
                {
                    APS_ERR("enable als fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_ALS, &obj->enable);
            }
            else
            {
                if(err = ltr558_enable_als(obj->client, false))
                {
                    APS_ERR("disable als fail: %d\n", err);
                    goto err_out;
                }
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }
            break;

        case ALSPS_GET_ALS_MODE:
            enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA:
            if(err = ltr558_read_data_als(obj->client, &obj->als))
            {
                goto err_out;
            }

            dat = obj->als;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:
            if(err = ltr558_read_data_als(obj->client, &obj->als))
            {
                goto err_out;
            }

            dat = obj->als;	// & 0x3f;//modified by mayq
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
//Gionee mali add calibration for ltr558 2012-9-28 begin
#if defined(GN_PS_NEED_CALIBRATION)
        case ALSPS_SET_PS_CALI:
            /*case ALSPS_SET_PS_CALI:*/
            dat = (void __user*)arg;
            if(dat == NULL)
            {
                APS_LOG("dat == NULL\n");
                err = -EINVAL;
                break;    
            }
            
            if(copy_from_user(&ps_cali_temp, dat, sizeof(ps_cali_temp)))
            {
                APS_LOG("copy_from_user\n");
                err = -EFAULT;
                break;    
            }
           
            ltr558_WriteCalibration(&ps_cali_temp);
            APS_LOG("111111  ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close, ps_cali_temp.far_away,ps_cali_temp.valid);
            
            break;
        
        case ALSPS_GET_PS_CALI:
           
            if(err = ltr558_enable_ps(obj->client, true))
            {
                APS_ERR("ltr558 ioctl enable ps fail: %d\n", err);
            }
            
            msleep(200);
            
            err = ltr558_read_data_for_cali(obj->client, &ps_cali_temp);
            if(err)
            {
                goto err_out;
            }
            
            if(err = ltr558_enable_ps(client, false))
            {
                APS_ERR("ltr558 ioctl disable ps fail: %d\n", err);
            }
            
            if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
            {
                err = -EFAULT;
                goto err_out;
            }              
            APS_LOG("ltr558 ALSPS_GET_PS_CALI %d,%d,%d\t",ps_cali_temp.close, ps_cali_temp.far_away,ps_cali_temp.valid);
            break;
#endif
//Gionee mali add calibration for ltr558 2012-9-28 end
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations ltr558_fops = {
    .owner = THIS_MODULE,
    .open = ltr558_open,
    .release = ltr558_release,
    .unlocked_ioctl = ltr558_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr558_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &ltr558_fops,
};
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);
        if(err = ltr558_enable_als(client, false))
        {
            APS_ERR("disable als: %d\n", err);
            return err;
        }

        //Gionee yanggy 2012-07-25 add for psensor interrupt mode begin
        /*==========this part is necessary when use polling mode==========*/
#ifndef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
        atomic_set(&obj->ps_suspend, 1);
        if(err = ltr558_enable_ps(client, false))
        {
            APS_ERR("disable ps: %d\n", err);
            return err;
        }
#endif
        //Gionee yanggy 2012-07-25 add for psensor interrupt mode end
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_resume(struct i2c_client *client)
{
    struct ltr558_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if(err = ltr558_enable_als(client, true))
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }

    //Gionee yanggy 2012-07-25 add for psensor interrupt mode begin
    /*==========this part is necessary when use polling mode======*/
#ifndef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    atomic_set(&obj->ps_suspend, 0);
    if(test_bit(CMC_BIT_PS, &obj->enable))
    {
        if(err = ltr558_enable_ps(client, true))
        {
            APS_ERR("enable ps fail: %d\n", err);
        }
    }
#endif
    //Gionee yanggy 2012-07-25 add for psensor interrupt mode

    return 0;
}
/*----------------------------------------------------------------------------*/
static void ltr558_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr558_priv *obj = container_of(h, struct ltr558_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
    if(err = ltr558_enable_als(obj->client, false))
    {
        APS_ERR("disable als fail: %d\n", err);
    }
}
/*----------------------------------------------------------------------------*/
static void ltr558_late_resume(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr558_priv *obj = container_of(h, struct ltr558_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if(err = ltr558_enable_als(obj->client, true))
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }

}

int ltr558_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct ltr558_priv *obj = (struct ltr558_priv *)self;

    //APS_FUN(f);
    APS_LOG("ltr558_ps_operate command:%d\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if(value)
                {
                    if(err = ltr558_enable_ps(obj->client, true))
                    {
                        APS_ERR("enable ps fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    if(err = ltr558_enable_ps(obj->client, false))
                    {
                        APS_ERR("disable ps fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_PS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            } else {
                sensor_data = (hwm_sensor_data *)buff_out;
                if (err = ltr558_read_data_ps(obj->client, &obj->ps)) {
                    APS_ERR("SENSOR_GET_DATA^^^^^^^^^^!\n");
                    err = -1;
                    break;
                } else { 
                    while(-1 == ltr558_get_ps_value(obj, obj->ps)) {
                        ltr558_read_data_ps(obj->client, &obj->ps);
                        msleep(50);
                    }
                    sensor_data->values[0] = ltr558_get_ps_value(obj, obj->ps);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    APS_LOG("fwq get ps data =%d\n",sensor_data->values[0]);
                }
            }
            break;
        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

int ltr558_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct ltr558_priv *obj = (struct ltr558_priv *)self;

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if(value)
                {
                    if(err = ltr558_enable_als(obj->client, true))
                    {
                        APS_ERR("enable als fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_ALS, &obj->enable);
                }
                else
                {
                    if(err = ltr558_enable_als(obj->client, false))
                    {
                        APS_ERR("disable als fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("ltr558_als_operate get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;

                if(err = ltr558_read_data_als(obj->client, &obj->als))
                {
                    err = -1;;
                }
                else
                {
                    while(-1 == ltr558_get_als_value(obj, obj->als))
                    {
                        ltr558_read_data_als(obj->client, &obj->als);
                        msleep(50);
                    }
                    
                    ltr558_read_data_ps(obj->client, &obj->ps);
                    ltr558_get_ps_value(obj, obj->ps);
                    sensor_data->values[0] = ltr558_get_als_value(obj, obj->als);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    APS_LOG("ltr558_als_operate get als data =%d\n",sensor_data->values[0]);
                }
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
static int ltr558_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    APS_FUN();
    strcpy(info->type, LTR558_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr558_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ltr558_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0;
    APS_FUN();

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    memset(obj, 0, sizeof(*obj));
    ltr558_obj = obj;

    obj->hw = get_cust_alsps_hw_ltr558();
    ltr558_obj->polling = obj->hw->polling_mode;

#ifdef GN_MTK_BSP_ALSPS_INTERRUPT_MODE
    INIT_DELAYED_WORK(&obj->eint_work, ltr558_eint_work);
#endif
    obj->client = client;
    i2c_set_clientdata(client, obj);
    atomic_set(&obj->als_debounce, 1000);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 200);
    //    atomic_set(&obj->ps_deb_on, 0);
    //    atomic_set(&obj->ps_deb_end, 0);
    //    atomic_set(&obj->ps_mask, 0);
    //    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);

    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
    //pre set ps threshold
    obj->ps_thd_val = obj->hw->ps_threshold;
    //pre set window loss
    obj->als_widow_loss = obj->hw->als_window_loss;

    ltr558_i2c_client = client;

    if(err = ltr558_init_client(client))
    {
        goto exit_init_failed;
    }
    /*
       if(err = ltr558_enable_als(client, false))
       {
       APS_ERR("disable als fail: %d\n", err);
       }
       if(err = ltr558_enable_ps(client, false))
       {
       APS_ERR("disable ps fail: %d\n", err);
       }
     */
    if(err = misc_register(&ltr558_device))
    {
        APS_ERR("ltr558_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    //Gionee yanggy 2012-07-25 modify for psensor compability begin 
#ifdef MTK_AUTO_DETECT_ALSPS
    if(err = ltr558_create_attr(&(ltr558_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
#else
    if(err = ltr558_create_attr(&ltr558_alsps_driver.driver))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
#endif
    //Gionee yanggy 2012-07-25 modify for psensor compability end
    /*---------------ps_obj init-----------------*/
    obj_ps.self = ltr558_obj;
    APS_LOG("obj->hw->polling_mode:%d\n",obj->hw->polling_mode);
    if(1 == obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
    }
    obj_ps.sensor_operate = ltr558_ps_operate;
    if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    /*-------------als_obj init---------------- */
    obj_als.self = ltr558_obj;
    if(1 == obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_LOG("polling mode\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_LOG("interrupt mode\n");
    }
    obj_als.sensor_operate = ltr558_als_operate;
    if(err = hwmsen_attach(ID_LIGHT, &obj_als))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
        obj->early_drv.suspend = ltr558_early_suspend,
        obj->early_drv.resume = ltr558_late_resume,
        register_early_suspend(&obj->early_drv);
#endif

    APS_LOG("%s: OK\n", __func__);
    ltr558_init_flag = 0;
    return 0;

exit_create_attr_failed:
    misc_deregister(&ltr558_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
exit_kfree:
    kfree(obj);
exit:
    ltr558_i2c_client = NULL;
    APS_ERR("%s: err = %d\n", __func__, err);

    ltr558_init_flag = -1;
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr558_i2c_remove(struct i2c_client *client)
{
    int err;

    //Gionee yanggy 2012-07-25 modify for psensor compability begin
#ifdef MTK_AUTO_DETECT_ALSPS
    if(err = ltr558_delete_attr(&(ltr558_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("ltr558_delete_attr fail: %d\n", err);
    }
#else
    if(err = ltr558_delete_attr(&ltr558_alsps_driver.driver))
    {
        APS_ERR("ltr558_delete_attr fail: %d\n", err);
    }
#endif
    //Gionee yanggy 2012-07-25 modify for psensor compability end
    if(err = misc_deregister(&ltr558_device))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    ltr558_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}
/*----------------------------------------------------------------------------*/
//Gionee yanggy 2012-07-23 modify for psensor copability begin
#ifdef MTK_AUTO_DETECT_ALSPS
static int ltr558_local_init(void)
{
    struct acc_hw *hw = get_cust_alsps_hw_ltr558();
    ltr558_power(hw, 1);
    if(i2c_add_driver(&ltr558_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    if(-1 == ltr558_init_flag)
    {
        return -1;
    }

    return 0;
}
#else
static int ltr558_probe(struct platform_device *pdev)
{
    struct alsps_hw *hw = get_cust_alsps_hw_ltr558();      
    ltr558_power(hw, 1);
    if (i2c_add_driver(&ltr558_i2c_driver)) {
        APS_ERR("i2c_add_driver add driver error %d\n",__LINE__);
        return -1;
    }
    return 0;
}
#endif
//Gionee yanggy 2012-07-23 modify for psensor compability end
/*----------------------------------------------------------------------------*/
static int ltr558_remove(void)
{
    struct alsps_hw *hw = get_cust_alsps_hw_ltr558();
    APS_FUN();
    ltr558_power(hw, 0);
    i2c_del_driver(&ltr558_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
//Gionee yanggy 2012-07-23 modify for psensor compability begin
#ifndef MTK_AUTO_DETECT_ALSPS
static struct platform_driver ltr558_alsps_driver = {
    .probe	= ltr558_probe,
    .remove	= ltr558_remove,
    .driver	= {
        .name	= "als_ps",
    }
};
#endif
//Gionee yanggy 2012-07-23 modify for psensor compability end
/*----------------------------------------------------------------------------*/
static int __init ltr558_init(void)
{
    APS_FUN();
    i2c_register_board_info(3, &i2c_ltr558, 1);  
    //Gionee yanggy 2012-07-23 modify for psensor compability begin
#ifdef MTK_AUTO_DETECT_ALSPS
//    if(hwmsen_psensor_add(&ltr558_init_info))
//    {
//        APS_ERR("ltr558_init failed to register driver\n");
//        return -ENODEV;
//    }
#else
    if (platform_driver_register(&ltr558_alsps_driver)) {
        APS_ERR("ltr558_init failed to register driver\n");
        return -ENODEV;
    }
#endif
    //Gionee yanggy 2012-07-23 modify for psensor compability end
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr558_exit(void)
{
    APS_FUN();

#ifdef MTK_AUTO_DETECT_ALSPS
//    hwmsen_psensor_del(&ltr558_init_info);
#else
    platform_driver_unregister(&ltr558_alsps_driver);
#endif
}

/*----------------------------------------------------------------------------*/
module_init(ltr558_init);
module_exit(ltr558_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Chen Qiyan(chenqy@gionee.com");
MODULE_DESCRIPTION("LTR558 light sensor & p sensor driver");
MODULE_LICENSE("GPL");