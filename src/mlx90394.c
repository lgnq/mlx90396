/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-20     lgnq         the first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "mlx90394.h"

#include <string.h>
#include <stdlib.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME "MLX90396"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define CRC8_POLYNOMIAL 0x2F

rt_uint8_t cal_bytes_crc8(uint8_t *data, uint8_t len)
{
    rt_uint8_t crc8 = 0x00; // 初始CRC值
    rt_uint8_t i;

    while (len--)
    {
        crc8 ^= *data++; // 与数据进行异或

        for (i = 8; i > 0; --i)
        {
            crc8 = (crc8 & 0x80) ? (crc8 << 1) ^ CRC8_POLYNOMIAL : (crc8 << 1);
        }
    }

    return crc8;
}

/**
 * This function reads the value of register for mlx90394
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90394
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx90394_mem_direct_read(struct mlx90394_device *dev, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_RD;        /* Read flag */
        msgs.buf   = recv_buf;         /* Read data pointer */
        msgs.len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            LOG_E("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

/**
 * This function reads the value of register for mlx90394
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90394
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
static rt_err_t mlx90394_mem_read(struct mlx90394_device *dev, rt_uint8_t start_addr, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[5] = {(dev->i2c_addr)<<1, 0x80, 0x50, start_addr, 0xE7};

    send_buf[4] = cal_bytes_crc8(send_buf, 4);

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &send_buf[1];     /* Write data pointer */
        msgs[0].len   = 4;                /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            LOG_E("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

static rt_err_t mlx90396_read_register(struct mlx90394_device *dev, rt_uint8_t start_addr, rt_uint8_t *recv_buf, rt_uint8_t recv_len)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[5] = {(dev->i2c_addr)<<1, 0x80, 0x50, start_addr, 0xE7};

    send_buf[4] = cal_bytes_crc8(send_buf, 4);

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs[2];

        msgs[0].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &send_buf[1];     /* Write data pointer */
        msgs[0].len   = 4;                /* Number of bytes write */

        msgs[1].addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = recv_buf;         /* Read data pointer */
        msgs[1].len   = recv_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            LOG_E("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

static rt_err_t mlx90394_write_register(struct mlx90394_device *dev, rt_uint8_t reg, rt_uint16_t data)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[] = {dev->i2c_addr<<1, 0x80, 0x60, reg, (data&0xFF00)>>8, data&0xFF, 0x00};

    send_buf[6] = cal_bytes_crc8(send_buf, 6);

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = sizeof(send_buf); /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            LOG_E("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

/**
 * This function reads the value of register for mlx90394
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mlx90394
 * @param val read data pointer
 *
 * @return the reading status, RT_EOK represents reading the value of register successfully.
 */
//send_buf = start register address + data1 + data2 + ...
static rt_err_t mlx90394_mem_write(struct mlx90394_device *dev, rt_uint8_t *send_buf, rt_uint8_t send_len)
{
    rt_err_t res = RT_EOK;

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        struct rt_i2c_msg msgs;

        msgs.addr  = dev->i2c_addr;    /* I2C Slave address */
        msgs.flags = RT_I2C_WR;        /* Read flag */
        msgs.buf   = send_buf;         /* Read data pointer */
        msgs.len   = send_len;         /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            LOG_E("rt_i2c_transfer error\r\n");
            res = -RT_ERROR;
        }
#endif
    }

    return res;
}

rt_err_t mlx90394_reset(struct mlx90394_device *dev)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MLX90394_ADDR_RST;
    send_buf[1] = 0x06;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("Reset error\r\n");
    }
    else
    {
        LOG_I("Reset MLX90394 is done\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_cid(struct mlx90394_device *dev, rt_uint8_t *cid)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CID, cid, 1);
    if (res != RT_EOK)
    {
        LOG_E("Read CID is error\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_did(struct mlx90394_device *dev, rt_uint8_t *did)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_DID, did, 1);
    if (res != RT_EOK)
    {
        LOG_E("Read DID is error\r\n");
    }

    return res;
}

static rt_err_t mlx90394_get_stat1(struct mlx90394_device *dev, union mlx90394_stat1 *stat1)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_STAT1, (rt_uint8_t *)stat1, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("STAT1 = 0x%x, DRDY = 0x%x\r\n", stat1->byte_val, stat1->drdy);
    }

    return res;
}

static rt_err_t mlx90394_get_stat2(struct mlx90394_device *dev, union mlx90394_stat2 *stat2)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_STAT2, (rt_uint8_t *)stat2, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("STAT2 = 0x%x\r\n", stat2->byte_val);
    }

    return res;
}

rt_err_t mlx90394_get_ctrl1(struct mlx90394_device *dev, mlx90394_ctrl1_t *ctrl1)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, (rt_uint8_t *)ctrl1, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("CTRL1 = 0x%x\r\n", ctrl1->byte_val);
    }

    return res;
}

rt_err_t mlx90394_set_ctrl1(struct mlx90394_device *dev, rt_uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MLX90394_ADDR_CTRL1;
    send_buf[1] = val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("Set CTRL1 error\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_ctrl2(struct mlx90394_device *dev, mlx90394_ctrl2_t *ctrl2)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL2, (rt_uint8_t *)ctrl2, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("CTRL2 = 0x%x\r\n", ctrl2->byte_val);
    }

    return res;
}

rt_err_t mlx90394_set_ctrl2(struct mlx90394_device *dev, rt_uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MLX90394_ADDR_CTRL2;
    send_buf[1] = val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("Set CTRL2 error\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_ctrl3(struct mlx90394_device *dev, mlx90394_ctrl3_t *ctrl3)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, (rt_uint8_t *)ctrl3, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("CTRL3 = 0x%x\r\n", ctrl3->byte_val);
    }

    return res;
}

rt_err_t mlx90394_set_ctrl3(struct mlx90394_device *dev, rt_uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MLX90394_ADDR_CTRL3;
    send_buf[1] = val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("Set CTRL3 error\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_ctrl4(struct mlx90394_device *dev, mlx90394_ctrl4_t *ctrl4)
{
    rt_err_t res = RT_EOK;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL4, (rt_uint8_t *)ctrl4, 1);
    if (res != RT_EOK)
    {
        LOG_E("error\r\n");
    }
    else
    {
        LOG_D("CTRL4 = 0x%x\r\n", ctrl4->byte_val);
    }

    return res;
}

rt_err_t mlx90394_set_ctrl4(struct mlx90394_device *dev, rt_uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    send_buf[0] = MLX90394_ADDR_CTRL4;
    send_buf[1] = val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("Set CTRL4 error\r\n");
    }

    return res;
}

static rt_bool_t mlx90394_is_data_ready(struct mlx90394_device *dev)
{
    union mlx90394_stat1 stat1;

    mlx90394_get_stat1(dev, &stat1);
    if (stat1.drdy)
    {
        return RT_TRUE;
    }
    else
    {
        return RT_FALSE;
    }
}

rt_err_t mlx90394_get_temperature(struct mlx90394_device *dev, float *t)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[2];

    while (mlx90394_is_data_ready(dev) == RT_FALSE)
    {
        rt_thread_delay(100);
    }

    res = mlx90394_mem_read(dev, 0x8, recv_buf, 2);
    if (res == RT_EOK)
    {
        *t = (float)(((rt_int16_t)recv_buf[1] << 8 ) | recv_buf[0]) / MAGNETO10_TEMPERATURE_RES;
    }

    return res;
}

rt_err_t mlx90394_set_xonoff(struct mlx90394_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl1_t ctrl1;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, &ctrl1.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL1 error\r\n");
        return res;
    }

    ctrl1.x_en = onoff;

    send_buf[0] = MLX90394_ADDR_CTRL1;
    send_buf[1] = ctrl1.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set x enabled failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_set_yonoff(struct mlx90394_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl1_t ctrl1;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, &ctrl1.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL1 error\r\n");
        return res;
    }

    ctrl1.y_en = onoff;

    send_buf[0] = MLX90394_ADDR_CTRL1;
    send_buf[1] = ctrl1.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set y enabled failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_set_zonoff(struct mlx90394_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl1_t ctrl1;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, &ctrl1.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL1 error\r\n");
        return res;
    }

    ctrl1.z_en = onoff;

    send_buf[0] = MLX90394_ADDR_CTRL1;
    send_buf[1] = ctrl1.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set z enabled failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_set_tonoff(struct mlx90394_device *dev, rt_uint8_t onoff)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl4_t ctrl4;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL4, &ctrl4.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL4 error\r\n");
        return res;
    }

    ctrl4.t_en = onoff;

    send_buf[0] = MLX90394_ADDR_CTRL4;
    send_buf[1] = ctrl4.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set t enabled failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_mode(struct mlx90394_device *dev, rt_uint8_t *mode)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl1_t ctrl1;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, &ctrl1.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL1 failed\r\n");
        return res;
    }

    *mode = ctrl1.mode;

    return res;
}

rt_err_t mlx90394_set_mode(struct mlx90394_device *dev, enum mlx90394_mode application_mode)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl1_t ctrl1;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL1, &ctrl1.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL1 failed\r\n");
        return res;
    }

    ctrl1.mode = application_mode;

    send_buf[0] = MLX90394_ADDR_CTRL1;
    send_buf[1] = ctrl1.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set application mode error\r\n");
    }
    else
    {
        switch (application_mode)
        {
        case POWER_DOWN_MODE:
            LOG_D("POWER_DOWN_MODE\r\n");
            break;
        case SINGLE_MEASUREMENT_MODE:
            LOG_D("SINGLE_MEASUREMENT_MODE\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_10HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_10HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_15HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_15HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_50HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_50HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_100HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_100HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_200HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_200HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_500HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_500HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_700HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_700HZ\r\n");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_1100HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_1100HZ");
            break;
        case CONTINUOUS_MEASUREMENT_MODE_1400HZ:
            LOG_D("CONTINUOUS_MEASUREMENT_MODE_1400HZ");
            break;
        default:
            LOG_D("unknown application mode\r\n");
            break;
        }
    }

    return res;
}

rt_err_t mlx90394_get_range(struct mlx90394_device *dev, rt_uint8_t *range)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl2_t ctrl2;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL2, &ctrl2.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL2 failed\r\n");
        return res;
    }

    *range = ctrl2.range_config;

    return res;
}

rt_err_t mlx90394_set_range(struct mlx90394_device *dev, enum mlx90394_range range)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl2_t ctrl2;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL2, &ctrl2.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL2 failed\r\n");
        return res;
    }

    ctrl2.range_config = range;

    send_buf[0] = MLX90394_ADDR_CTRL2;
    send_buf[1] = ctrl2.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set range configuration failed\r\n");
    }
    else
    {
        switch (range)
        {
        case LOW_CURRENT_HIGH_RANGE:
            LOG_D("LOW_CURRENT_HIGH_RANGE\r\n");
            break;
        case LOW_NOISE_HIGH_RANGE:
            LOG_D("LOW_NOISE_HIGH_RANGE\r\n");
            break;
        case LOW_NOISE_HIGH_SENSITIVITY:
            LOG_D("LOW_NOISE_HIGH_SENSITIVITY\r\n");
            break;
        default:
            LOG_D("unknown range configuration\r\n");
            break;
        }
    }

    return res;
}

rt_err_t mlx90394_get_osr_hall(struct mlx90394_device *dev, uint8_t *val)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    *val = ctrl3.osr_hall;

    return res;
}

rt_err_t mlx90394_set_osr_hall(struct mlx90394_device *dev, uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    ctrl3.osr_hall = val;

    send_buf[0] = MLX90394_ADDR_CTRL3;
    send_buf[1] = ctrl3.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set OST_HALL failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_osr_temp(struct mlx90394_device *dev, uint8_t *val)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    *val = ctrl3.osr_temp;

    return res;
}

rt_err_t mlx90394_set_osr_temp(struct mlx90394_device *dev, uint8_t val)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    ctrl3.osr_temp = val;

    send_buf[0] = MLX90394_ADDR_CTRL3;
    send_buf[1] = ctrl3.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set OSR_TEMP failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_xyz(struct mlx90394_device *dev, struct mlx90394_xyz *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    res = mlx90394_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = recv_buf[1]<<8 | recv_buf[0];
        xyz->y = recv_buf[3]<<8 | recv_buf[2];
        xyz->z = recv_buf[5]<<8 | recv_buf[4];
    }

    return res;
}

static rt_err_t mlx90394_get_sensitivity(struct mlx90394_device *dev, float *sensitivity)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t range;

    res = mlx90394_get_range(dev, &range);
    if (res != RT_EOK)
    {
        LOG_E("read RANGE failed\r\n");
        return res;
    }

    switch (range)
    {
    case LOW_CURRENT_HIGH_RANGE:
        *sensitivity = 1.5;
        break;
    case LOW_NOISE_HIGH_RANGE:
        *sensitivity = 1.5;
        break;
    case LOW_NOISE_HIGH_SENSITIVITY:
        *sensitivity = 0.15;
        break;
    default:
        LOG_E("unkonwn magnetic sensor measurement range\r\n");
        break;
    }

    return res;
}

rt_err_t mlx90394_get_xyz_flux(struct mlx90394_device *dev, struct mlx90394_xyz_flux *xyz)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t recv_buf[6];

    res = mlx90394_mem_read(dev, 0x1, recv_buf, 6);
    if (res == RT_EOK)
    {
        xyz->x = (dev->sensitivity) * ((rt_int16_t)((recv_buf[1] << 8) | recv_buf[0]));
        xyz->y = (dev->sensitivity) * ((rt_int16_t)((recv_buf[3] << 8) | recv_buf[2]));
        xyz->z = (dev->sensitivity) * ((rt_int16_t)((recv_buf[5] << 8) | recv_buf[4]));
    }

    return res;
}

rt_err_t mlx90394_get_woc_mode(struct mlx90394_device *dev, uint8_t *mode)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl2_t ctrl2;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL2, &ctrl2.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL2 failed\r\n");
        return res;
    }

    *mode = ctrl2.woc_mode;

    return res;
}

rt_err_t mlx90394_set_woc_mode(struct mlx90394_device *dev, uint8_t mode)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl2_t ctrl2;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL2, &ctrl2.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL2 failed\r\n");
        return res;
    }

    ctrl2.woc_mode = mode;

    send_buf[0] = MLX90394_ADDR_CTRL2;
    send_buf[1] = ctrl2.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set WOC_MODE failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_dig_filt_xy(struct mlx90394_device *dev, uint8_t *dig_filt)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    *dig_filt = ctrl3.dig_filt_hall_xy;

    return res;
}

rt_err_t mlx90394_set_dig_filt_xy(struct mlx90394_device *dev, uint8_t dig_filt)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    ctrl3.dig_filt_hall_xy = dig_filt;

    send_buf[0] = MLX90394_ADDR_CTRL3;
    send_buf[1] = ctrl3.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set DIG_FILT_XY failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_dig_filt_z(struct mlx90394_device *dev, uint8_t *dig_filt)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl4_t ctrl4;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL4, &ctrl4.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL4 failed\r\n");
        return res;
    }

    *dig_filt = ctrl4.dig_filt_hall_z;

    return res;
}

rt_err_t mlx90394_set_dig_filt_z(struct mlx90394_device *dev, uint8_t dig_filt)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl4_t ctrl4;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL4, &ctrl4.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL4 failed\r\n");
        return res;
    }

    ctrl4.dig_filt_hall_z = dig_filt;

    send_buf[0] = MLX90394_ADDR_CTRL4;
    send_buf[1] = ctrl4.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set DIG_FILT_Z failed\r\n");
    }

    return res;
}

rt_err_t mlx90394_get_dig_filt_t(struct mlx90394_device *dev, uint8_t *dig_filt)
{
    rt_err_t res = RT_EOK;
    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    *dig_filt = ctrl3.dig_filt_temp;

    return res;
}

rt_err_t mlx90394_set_dig_filt_t(struct mlx90394_device *dev, uint8_t dig_filt)
{
    rt_err_t res = RT_EOK;
    rt_uint8_t send_buf[2];

    mlx90394_ctrl3_t ctrl3;

    res = mlx90394_mem_read(dev, MLX90394_ADDR_CTRL3, &ctrl3.byte_val, 1);
    if (res != RT_EOK)
    {
        LOG_E("read CTRL3 failed\r\n");
        return res;
    }

    ctrl3.dig_filt_temp = dig_filt;

    send_buf[0] = MLX90394_ADDR_CTRL3;
    send_buf[1] = ctrl3.byte_val;
    res = mlx90394_mem_write(dev, send_buf, 2);
    if (res != RT_EOK)
    {
        LOG_E("set DIG_FILT_T failed\r\n");
    }

    return res;
}

void mlx90394_setup(struct mlx90394_device *dev)
{
//    mlx90394_reset(dev);

//    rt_thread_delay(10000);

//    mlx90394_set_gain_sel(dev, 4);
//    mlx90394_set_resolution(dev, 0, 0, 0);
}

/**
 * This function gets the raw data of mlx90394
 *
 * @param dev the pointer of device driver structure
 * @param xyz the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK represents  reading the data successfully.
 */
static rt_err_t mlx90394_continuous_measurement(struct mlx90394_device *dev, struct mlx90394_xyz *xyz, rt_uint16_t freq)
{
    rt_uint8_t status = RT_EOK;
    union mlx90394_stat1 stat1;

    switch (freq)
    {
    case 10:
        LOG_D("10Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_10HZ);
        break;
    case 20:
        LOG_D("15Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_15HZ);
        break;
    case 50:
        LOG_D("50Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_50HZ);
        break;
    case 100:
        LOG_D("100Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_100HZ);
        break;
    case 200:
        LOG_D("200Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_200HZ);
        break;
    case 500:
        LOG_D("500Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_500HZ);
        break;
    case 700:
        LOG_D("700Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_700HZ);
        break;
    case 1100:
        LOG_D("1100Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_1100HZ);
        break;
    case 1400:
        LOG_D("1400Hz");
        status = mlx90394_set_mode(dev, CONTINUOUS_MEASUREMENT_MODE_1400HZ);
        break;
    default:
        LOG_D("wrong frequency\r\n");
        break;
    }

    while (1)
    {
        status = mlx90394_get_stat1(dev, &stat1);

        if (stat1.drdy == 1)
        {
            status = mlx90394_get_xyz(dev, xyz);
            rt_kprintf("data%d,%d,%d\n", xyz->x, xyz->y, xyz->z);
        }

        rt_thread_delay(100);
    }

    return status;
}

rt_err_t mlx90394_single_measurement_raw(struct mlx90394_device *dev, struct mlx90394_xyz *xyz)
{
    rt_uint8_t status = RT_EOK;
    union mlx90394_stat1 stat1;

    status = mlx90394_set_mode(dev, SINGLE_MEASUREMENT_MODE);

    stat1.byte_val = 0;
    while (stat1.drdy == 0)
    {
        status = mlx90394_get_stat1(dev, &stat1);
        rt_thread_delay(50);
    }

    status = mlx90394_get_xyz(dev, xyz);

    return status;
}

rt_err_t mlx90394_single_measurement(struct mlx90394_device *dev, struct mlx90394_xyz_flux *xyz)
{
    rt_uint8_t status = RT_EOK;
    union mlx90394_stat1 stat1;

    status = mlx90394_set_mode(dev, SINGLE_MEASUREMENT_MODE);

    stat1.byte_val = 0;
    while (stat1.drdy == 0)
    {
        status = mlx90394_get_stat1(dev, &stat1);
        rt_thread_delay(100);
    }

    status = mlx90394_get_xyz_flux(dev, xyz);

    return status;
}

/**
 * This function initialize the mlx90394 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication
 *
 * @return the pointer of device driver structure, RT_NULL represents  initialization failed.
 */
struct mlx90394_device *mlx90394_init(const char *dev_name, rt_uint8_t param)
{
    struct mlx90394_device *dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mlx90394_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for mlx90394 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* find mlx90394 device at address: MLX90394_I2C_ADDRESS */
            dev->i2c_addr = MLX90394_I2C_ADDRESS;
        }

        rt_uint8_t id[4];

        for (rt_uint8_t i=0; i<128; i++)
        {
            dev->i2c_addr = i;

            if (mlx90396_read_register(dev, 0x3B, id, 4) == RT_EOK)
            {
//                if (id[0] == 0x06)
                {
                    LOG_I("Device i2c address is:'0x%x'!\r\n", dev->i2c_addr);

                    LOG_I("STATUS is 0x%x\r\n", id[0]);
                    LOG_I("XPOS is 0x%x\r\n", id[1]);
                    LOG_I("YPOS is 0x%x\r\n", id[2]);
                    LOG_I("CRC is 0x%x\r\n", id[3]);

                    return dev;
                }
            }
        }

        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;

#endif        
    }
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }

    return RT_NULL;
}

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90394_deinit(struct mlx90394_device *dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

static void mlx90394(int argc, char **argv)
{
    static struct mlx90394_device *dev = RT_NULL;

    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        LOG_I("\n");
        LOG_I("mlx90394 [OPTION] [PARAM]\n");
        LOG_I("         probe <dev_name>      Probe mlx90394 by given name, ex:i2c2\n");
        LOG_I("         id                    Print CID and DID\n");
        LOG_I("         stat1                 Print stat1\n");
        LOG_I("         read [num]            read [num] times mlx90394\n");
        LOG_I("                               num default 5\n");
        return;
    }
    else
    {
        if (!strcmp(argv[1], "probe"))
        {
            if (dev)
            {
                mlx90394_deinit(dev);
            }

            if (argc == 2)
                dev = mlx90394_init("i2c2", RT_NULL);
            else if (argc == 3)
                dev = mlx90394_init(argv[2], RT_NULL);
        }
        else if (dev == RT_NULL)
        {
            LOG_E("Please probe mlx90394 first!\n");
            return;
        }
        else if (!strcmp(argv[1], "id"))
        {
            rt_uint8_t id[2];
            rt_uint8_t start_addr = 10;
            rt_uint8_t len = 2;

            mlx90394_mem_read(dev, start_addr, id, len);
            LOG_I("CID = 0x%x\r\n", id[0]);
            LOG_I("DID = 0x%x\r\n", id[1]);
        }
        else if (!strcmp(argv[1], "stat1"))
        {
            union mlx90394_stat1 stat1;

            mlx90394_get_stat1(dev, &stat1);
        }
        else if (!strcmp(argv[1], "mode"))
        {
            mlx90394_set_mode(dev, atoi(argv[2]));
        }
        else if (!strcmp(argv[1], "t"))
        {
            float t;

            mlx90394_set_mode(dev, SINGLE_MEASUREMENT_MODE);
            mlx90394_get_temperature(dev, &t);
            LOG_I("t = %d.%d\r\n", (rt_int16_t)t, (rt_uint16_t)(t*100)%100);
        }
        else if (!strcmp(argv[1], "rr"))
        {
            rt_uint8_t val;
            mlx90394_mem_read(dev, atoi(argv[2]), &val, 1);

            LOG_I("Reading REG[%d] = 0x%x...\r\n", atoi(argv[2]), val);
        }
        else if (!strcmp(argv[1], "setup"))
        {
            mlx90394_setup(dev);
        }
        else if (!strcmp(argv[1], "xyz"))
        {
            struct mlx90394_xyz_flux xyz;

            mlx90394_single_measurement(dev, &xyz);

            LOG_I("x = %d.%d\r\n", (rt_int16_t)xyz.x, (rt_int16_t)(xyz.x*10)%10);
            LOG_I("y = %d.%d\r\n", (rt_int16_t)xyz.y, (rt_int16_t)(xyz.y*10)%10);
            LOG_I("z = %d.%d\r\n", (rt_int16_t)xyz.z, (rt_int16_t)(xyz.z*10)%10);
        }
        else if (!strcmp(argv[1], "continuous"))
        {
            struct mlx90394_xyz xyz;

            if (argc == 2)
                mlx90394_continuous_measurement(dev, &xyz, 500);
            else if (argc == 3)
                mlx90394_continuous_measurement(dev, &xyz, atoi(argv[2]));
        }
        else
        {
            LOG_E("Unknown command, please enter 'mlx90394' get help information!\n");
        }
    }
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(mlx90394, mlx90394 sensor function);
#endif

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90394, mlx90394 sensor function);
#endif
