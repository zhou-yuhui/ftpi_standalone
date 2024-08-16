/*
 * Copyright : (C) 2022 Phytium Information Technology, Inc.
 * All Rights Reserved.
 *
 * This program is OPEN SOURCE software: you can redistribute it and/or modify it
 * under the terms of the Phytium Public License as published by the Phytium Technology Co.,Ltd,
 * either version 1.0 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the Phytium Public License for more details.
 *
 *
 * FilePath: rpmsg-echo.c
 * Date: 2022-03-08 22:00:15
 * LastEditTime: 2022-03-09 10:01:19
 * Description:  This is a sample demonstration application that showcases usage of rpmsg
 *  This application is meant to run on the remote CPU running baremetal code.
 *  This application echoes back data that was sent to it by the master core.
 *
 * Modify History:
 *  Ver   Who        Date         Changes
 * ----- ------     --------    --------------------------------------
 * 1.0   huanghe    2022/06/20      first release
 */

/***************************** Include Files *********************************/

#include <stdio.h>
#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include "platform_info.h"
#include "rpmsg_service.h"
#include <metal/sleep.h>
#include "rsc_table.h"
#include "shell.h"
#include "fcache.h"
#include "fdebug.h"

#include "fi2c.h"
#include <stdio.h>
#include "ftypes.h"
#include "fassert.h"
#include "ferror_code.h"
#include "fmio.h"
#include "fmio_hw.h"
#include "fio_mux.h"

#include "sdkconfig.h"
#ifndef SDK_CONFIG_H__
    #warning "Please include sdkconfig.h"
#endif

#include <string.h>
#include <stdio.h>
#include "strto.h"

#include "ftypes.h"
#include "fdebug.h"
#include "fassert.h"
#include "fsleep.h"
#include "fio_mux.h"

#include "fgpio_hw.h"

#include "pin_common.h"
#include "pin_gpio_low_level_example.h"

/************************** Constant Definitions *****************************/
/***************** Macros (Inline Functions) Definitions *********************/

#define SHUTDOWN_MSG                0xEF56A55A
#define     ECHO_DEV_SLAVE_DEBUG_TAG "    SLAVE_01"
#define     ECHO_DEV_SLAVE_DEBUG_I(format, ...) FT_DEBUG_PRINT_I( ECHO_DEV_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define     ECHO_DEV_SLAVE_DEBUG_W(format, ...) FT_DEBUG_PRINT_W( ECHO_DEV_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define     ECHO_DEV_SLAVE_DEBUG_E(format, ...) FT_DEBUG_PRINT_E( ECHO_DEV_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)

#define FI2CMS_DEBUG_TAG "I2C-MASTER-SLAVE"
#define FI2CMS_ERROR(format, ...)   FT_DEBUG_PRINT_E(FI2CMS_DEBUG_TAG, format, ##__VA_ARGS__)

#define I2C_INSTANCE_ID 3  // 根据实际情况设置I2C实例ID
#define BH1750_ADDRESS 0x23  // 传感器地址

#define MASTER_MIO 8 // MIO的实例ID
#define FMIO_CLK_FREQ_HZ 50000000 // 50MHz时钟频率
#define FI2C_SPEED_STANDARD_RATE 100000

/************************** Variable Definitions *****************************/
static int shutdown_req = 0;

const char BH1750_POWER_ON = 0x01;
const char BH1750_RESET = 0x07;
const char BH1750_CONTINUOUS_HIGH_RES_MODE = 0x10;

static uintptr gpio_base = FGPIO2_BASE_ADDR;
static const u32 ctrl_id_base = FGPIO2_ID;
static u32 output_pin = (u32)FGPIO_PIN_6;

static uintptr gpio_fire = FGPIO2_BASE_ADDR;
static const u32 ctrl_id_fire = FGPIO2_ID;
static u32 input_pin = (u32)FGPIO_PIN_5;

/************************** Function Prototypes ******************************/

FI2c i2c_instance;
static FI2c master_device;
FMioCtrl mio_ctrl;
int lightLevel = 0;

int fire_flag = 0;

FError initialize_i2c() {
    FI2cConfig input_cfg;
    const FI2cConfig *config_p = NULL;
    FI2c *instance_p = &master_device;
    FError status = FI2C_SUCCESS;

    mio_ctrl.config = *FMioLookupConfig(MASTER_MIO);
    status = FMioFuncInit(&mio_ctrl, FMIO_FUNC_SET_I2C);
    if (status != FT_SUCCESS) {
        FI2CMS_ERROR("MIO initialize error.");
        return ERR_GENERAL;
    }

    FIOPadSetMioMux(MASTER_MIO);
    memset(instance_p, 0, sizeof(*instance_p));
    config_p = FI2cLookupConfig(3);/* get a default reference for MIO config */
    if (NULL == config_p)
    {
        FI2CMS_ERROR("Config of mio instance %d non found.\r\n", MASTER_MIO);
        return FI2C_ERR_INVAL_PARM;
    }
    input_cfg = *config_p;
    input_cfg.instance_id = MASTER_MIO;
    input_cfg.base_addr = FMioFuncGetAddress(&mio_ctrl, FMIO_FUNC_SET_I2C);
    input_cfg.irq_num = FMioFuncGetIrqNum(&mio_ctrl, FMIO_FUNC_SET_I2C);
    input_cfg.ref_clk_hz = FMIO_CLK_FREQ_HZ;
    // input_cfg.work_mode = FI2C_MASTER;
    input_cfg.slave_addr = 0; // 主机模式下不需要设置从地址
    input_cfg.speed_rate = FI2C_SPEED_STANDARD_RATE;
    status = FI2cCfgInitialize(&i2c_instance, &input_cfg);
    if (FI2C_SUCCESS != status)
    {
        FI2CMS_ERROR("Init mio master failed, ret: 0x%x", status);
        return status;
    }
    return FI2C_SUCCESS;
}



FError write_i2c_command(FI2c *i2c, uint8_t address, uint8_t command) {
    return FI2cMasterWritePoll(i2c, address, 0, &command, 1);
}

FError read_i2c_data(FI2c *i2c, uint8_t address, uint8_t *buffer, uint32_t length) {
    return FI2cMasterReadPoll(i2c, address, 0, buffer, length);
}

void FgpioExample(void)
{
    int ret = 0;
    int flag = 1;
    u32 reg_val;
    u32 set_level = FGPIO_PIN_HIGH;

    /* init pin */
    FIOPadSetGpioMux(ctrl_id_fire, input_pin);
    FIOPadSetGpioMux(ctrl_id_base, output_pin);

    reg_val = FGpioReadReg32(gpio_fire, FGPIO_SWPORTA_DDR_OFFSET); /* set direction */
    reg_val &= ~BIT(input_pin); /* 0-input */
    FGpioWriteReg32(gpio_fire, FGPIO_SWPORTA_DDR_OFFSET, reg_val);

    reg_val = FGpioReadReg32(gpio_base, FGPIO_SWPORTA_DDR_OFFSET); /* set direction */
    reg_val |= BIT(output_pin); /* 1-output */
    FGpioWriteReg32(gpio_base, FGPIO_SWPORTA_DDR_OFFSET, reg_val);
    
    // reg_val = FGpioReadReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET); /* set output pin to low-level */
    // reg_val &= ~BIT(output_pin);
    // FGpioWriteReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET, reg_val);

    

    reg_val = FGpioReadReg32(gpio_fire, FGPIO_EXT_PORTA_OFFSET); /* get input pin level */
    if (((BIT(input_pin) & reg_val) ? FGPIO_PIN_HIGH : FGPIO_PIN_LOW) == FGPIO_PIN_LOW)
    {
        fire_flag = 1;
        printf("fire.\n");
        reg_val = FGpioReadReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET); /* set output pin to high-level */
        reg_val |= BIT(output_pin); 
        FGpioWriteReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET, reg_val);
    }
    else
    {
        printf("no fire.\n");
        if(fire_flag == 1) {
            fire_flag = 0;
            reg_val = FGpioReadReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET); /* set output pin to low-level */
            reg_val &= ~BIT(output_pin);
            FGpioWriteReg32(gpio_base, FGPIO_SWPORTA_DR_OFFSET, reg_val);
            flag = 0;
        }
        
    }

    fsleep_millisec(100); /* delay 100ms */
}

/*-----------------------------------------------------------------------------*
 *  RPMSG endpoint callbacks
 *-----------------------------------------------------------------------------*/

static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    (void)priv;
    (void)src;

    /* On reception of a shutdown we signal the application to terminate */
    if ((*(unsigned int *)data) == SHUTDOWN_MSG)
    {
        ECHO_DEV_SLAVE_DEBUG_I("Shutdown message is received.\r\n");
        shutdown_req = 1;
        return RPMSG_SUCCESS;
    }

    // // 打开传感器
    // if (write_i2c_command(&i2c_instance, BH1750_ADDRESS, BH1750_POWER_ON) != FI2C_SUCCESS) {
    //     ECHO_DEV_SLAVE_DEBUG_E("Failed to power on the sensor\n");
    //     lightLevel++;
    //     // return -1;
    // }

    // // 重置传感器
    // if (write_i2c_command(&i2c_instance, BH1750_ADDRESS, BH1750_RESET) != FI2C_SUCCESS) {
    //     ECHO_DEV_SLAVE_DEBUG_E("Failed to reset the sensor\n");
    //     lightLevel++;
    //     // return -1;
    // }

    // // 设置为连续高分辨率模式
    // if (write_i2c_command(&i2c_instance, BH1750_ADDRESS, BH1750_CONTINUOUS_HIGH_RES_MODE) != FI2C_SUCCESS) {
    //     ECHO_DEV_SLAVE_DEBUG_E("Failed to set high resolution mode\n");
    //     return -1;
    // }

    // // 给传感器时间进行测量
    // for (volatile int i = 0; i < 1000000; i++);

    // // 读取光照强度
    // uint8_t buffer[2];
    // if (read_i2c_data(&i2c_instance, BH1750_ADDRESS, buffer, 2) != FI2C_SUCCESS) {
    //     ECHO_DEV_SLAVE_DEBUG_E("Failed to read light level from sensor\n");
    //     return -1;
    // }

    // int lightLevel = (buffer[0] << 8) | buffer[1];  // 将两个字节的数据合并为一个整数
    // ECHO_DEV_SLAVE_DEBUG_I("Light level: %d\n", lightLevel);

    char str[32];
    sprintf(str, "%d", lightLevel);
    data = (void*)str;
    len = 32;

#ifdef CONFIG_MEM_NORMAL
    FCacheDCacheInvalidateRange((intptr)data, len);
#endif
    
    /* Send data back to master */
    if (rpmsg_send(ept, data, len) < 0)
    {
        ECHO_DEV_SLAVE_DEBUG_E("rpmsg_send failed.\r\n");
    }

    return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
    (void)ept;
    ECHO_DEV_SLAVE_DEBUG_I("Unexpected remote endpoint destroy.\r\n");
    shutdown_req = 1;
}

/*-----------------------------------------------------------------------------*
 *  Application
 *-----------------------------------------------------------------------------*/
static int FRpmsgEchoApp(struct rpmsg_device *rdev, void *priv)
{
    int ret = 0;
    struct rpmsg_endpoint lept;
    shutdown_req = 0;
    /* Initialize RPMSG framework */
    ECHO_DEV_SLAVE_DEBUG_I("Try to create rpmsg endpoint.\r\n");
    printf("prinf: before gpiotest.\r\n");

    while(1) {
        FgpioExample();
    }
    // for(int i=10000;i>0;i--) {
    //     FgpioExample();
    // }
    printf("prinf: after gpiotest.\r\n");
    // initialize_i2c();
    // if (initialize_i2c() != FI2C_SUCCESS) {
    //     printf("prinf: I2C initialization failed.\r\n");
    //     lightLevel++;
    // }

    ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME, 0, RPMSG_ADDR_ANY, rpmsg_endpoint_cb, rpmsg_service_unbind);
    if (ret)
    {
        ECHO_DEV_SLAVE_DEBUG_E("Failed to create endpoint. %d \r\n", ret);
        return -1;
    }

    ECHO_DEV_SLAVE_DEBUG_I("Successfully created rpmsg endpoint.\r\n");
    // static u32 test_data = 1 ; 



    while (1)
    {
        platform_poll(priv);

        /* we got a shutdown request, exit */
        if (shutdown_req)
        {
            break;
        }
    }

    rpmsg_destroy_ept(&lept);

    return ret;
}

/*-----------------------------------------------------------------------------*
 *  Application entry point
 *-----------------------------------------------------------------------------*/

int FOpenampExample(void)
{
    int ret = 0;
    void *platform;
    struct rpmsg_device *rpdev;
    /* Initialize platform */
    ret = platform_init(0, NULL, &platform);
    if (ret)
    {
        ECHO_DEV_SLAVE_DEBUG_E("Failed to initialize platform.\r\n");
        platform_cleanup(platform);
        return -1;
    }

    rpdev = platform_create_rpmsg_vdev(platform, 0, VIRTIO_DEV_SLAVE, NULL, NULL);
    if (!rpdev)
    {
        ECHO_DEV_SLAVE_DEBUG_E("Failed to create rpmsg virtio device.\r\n");
        ret = platform_cleanup(platform);
        return ret;
    }
    
    ret = FRpmsgEchoApp(rpdev, platform);
    if (ret)
    {
        ECHO_DEV_SLAVE_DEBUG_E("Failed to running echoapp");
        return platform_cleanup(platform);
    }
    ECHO_DEV_SLAVE_DEBUG_I("Stopping application...");
    return ret;
}
