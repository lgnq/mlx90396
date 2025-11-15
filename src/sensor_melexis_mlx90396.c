/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include "sensor_melexis_mlx90396.h"
#include <stdlib.h>

#define DBG_TAG "sensor.melexis.mlx90396"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define mlx_dev ((struct mlx90396_device *)sensor->parent.user_data)

rt_uint16_t sample_freq = 100;

static struct mlx90396_device *_mlx90396_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    return mlx90396_init(intf->dev_name, i2c_addr);
}

rt_err_t mlx90396_get_info(rt_sensor_t sensor)
{
    rt_err_t res = RT_EOK;

//    rt_uint8_t cid;
//    rt_uint8_t did;
//
//    mlx90394_ctrl1_t ctrl1;
//    mlx90394_ctrl2_t ctrl2;
//    mlx90394_ctrl3_t ctrl3;
//    mlx90394_ctrl4_t ctrl4;

    struct mlx90396_device *dev = ((struct mlx90396_device *)sensor->parent.user_data);

    if (dev == RT_NULL)
    {
        LOG_E("Please probe mlx90396 first!\n");
        return -RT_ERROR;
    }

//    res  = mlx90394_get_cid(dev, &cid);
//    res += mlx90394_get_did(dev, &did);
//
//    res += mlx90394_get_ctrl1(dev, &ctrl1);
//    res += mlx90394_get_ctrl2(dev, &ctrl2);
//    res += mlx90394_get_ctrl3(dev, &ctrl3);
//    res += mlx90394_get_ctrl4(dev, &ctrl4);

    rt_kprintf("i2caddr:%x\n", dev->i2c_addr);
//    rt_kprintf("cid:%x\n", cid);
//    rt_kprintf("did:%x\n", did);
//    rt_kprintf("xonoff:%x\n", ctrl1.x_en);
//    rt_kprintf("yonoff:%x\n", ctrl1.y_en);
//    rt_kprintf("zonoff:%x\n", ctrl1.z_en);
//    rt_kprintf("tonoff:%x\n", ctrl4.t_en);
//    rt_kprintf("osrhall:%x\n", ctrl3.osr_hall);
//    rt_kprintf("osrtemp:%x\n", ctrl3.osr_temp);
//    rt_kprintf("mode:%x\n", ctrl1.mode);
//    rt_kprintf("range:%x\n", ctrl2.range_config);
//    rt_kprintf("digfltxy:%x\n", ctrl3.dig_filt_hall_xy);
//    rt_kprintf("digfltz:%x\n", ctrl4.dig_filt_hall_z);
//    rt_kprintf("digflttemp:%x\n", ctrl3.dig_filt_temp);
//    rt_kprintf("wocmode:%x\n", ctrl2.woc_mode);

    return res;
}

static rt_size_t _mlx90396_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        struct mlx90394_xyz xyz;

        if (mlx90396_single_measurement_legacy((struct mlx90396_device *)sensor->parent.user_data, &xyz) != RT_EOK)
        {
            LOG_E("mlx90396_single_measurement error\r\n");

            return 0;
        }

        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = xyz.x;
        data->data.mag.y = xyz.y;
        data->data.mag.z = xyz.z;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_size_t mlx90396_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mlx90396_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mlx90396_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = mlx_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    case RT_SENSOR_CTRL_USER_CMD_RESET:
        result = mlx90396_reset(mlx_dev);
        break;
    case RT_SENSOR_CTRL_USER_CMD_INFO:
        result = mlx90396_get_info(sensor);
        break;
    case RT_SENSOR_CTRL_USER_CMD_XONOFF:
        break;
    case RT_SENSOR_CTRL_USER_CMD_YONOFF:
        break;
    case RT_SENSOR_CTRL_USER_CMD_ZONOFF:
        break;
    case RT_SENSOR_CTRL_USER_CMD_TONOFF:
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_MODE:
        result = mlx90394_set_mode(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_RANGE:
        result = mlx90394_set_range(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_FILT_XY:
        result = mlx90394_set_dig_filt_xy(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_FILT_Z:
        result = mlx90394_set_dig_filt_z(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_FILT_T:
        result = mlx90394_set_dig_filt_t(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_WOC_MODE:
        result = mlx90394_set_woc_mode(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_OSR_HALL:
        result = mlx90394_set_osr_hall(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_OSR_TEMP:
        result = mlx90394_set_osr_temp(mlx_dev, *(rt_uint16_t *)args & 0xff);
        break;
    default:
        LOG_E("unknown RT_SENSOR_CTRL command\r\n");
        return -RT_ERROR;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mlx90396_fetch_data,
    mlx90396_control
};

int rt_hw_mlx90396_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mlx90396_device *mlx_dev_temp;
    rt_sensor_t sensor_mps = RT_NULL;

    mlx_dev_temp = _mlx90396_init(&cfg->intf);
    if (mlx_dev_temp == RT_NULL)
    {
        LOG_E("_mlx90396 init err!");
        goto __exit;
    }

    /* MPS sensor register */
    {
        sensor_mps = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mps == RT_NULL)
            goto __exit;

        sensor_mps->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mps->info.vendor     = RT_SENSOR_VENDOR_MELEXIS;
        sensor_mps->info.model      = "mlx90396";
        sensor_mps->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_mps->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mps->info.range_max  = 16000;
        sensor_mps->info.range_min  = 2000;
        sensor_mps->info.period_min = 5;

        rt_memcpy(&sensor_mps->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mps->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mps, name, RT_DEVICE_FLAG_RDWR, mlx_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (mlx_dev_temp)
        mlx90396_deinit(mlx_dev_temp);

    return -RT_ERROR;
}

int rt_hw_mlx90396_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = "i2c2";
    cfg.intf.user_data = (void *)MLX90394_I2C_ADDRESS;

    rt_hw_mlx90396_init("mps", &cfg);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_mlx90396_port);

static void read_mps_entry(void *parameter)
{
    rt_size_t res;
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        LOG_E("Can't find device:%s\n", parameter);
        return;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            LOG_E("device is already opened!\n");
        }
        else
        {
            LOG_E("open device failed!\n");
            return;
        }
    }

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            LOG_E("read data failed!size is %d\n", res);
            rt_device_close(dev);

            return;
        }
        else
        {
            rt_kprintf("data:%d,%d,%d\n", sensor_data.data.mag.x, sensor_data.data.mag.y, sensor_data.data.mag.z);
//            rt_kprintf("data:%d.%d,%d.%d,%d.%d\n", (rt_int16_t)xyz.x, (rt_uint16_t)(xyz.x * 100) % 100,
//                                                   (rt_int16_t)xyz.y, (rt_uint16_t)(xyz.y * 100) % 100,
//                                                   (rt_int16_t)xyz.z, (rt_uint16_t)(xyz.z * 100) % 100);
        }

        rt_thread_mdelay(sample_freq);
    }
}

rt_err_t mlx90396_measurement_onoff(int argc, char **argv)
{
    rt_thread_t mlx90396_thread;

    if (!strcmp(argv[1], "on"))
    {
        mlx90396_thread = rt_thread_create("mlx90396", read_mps_entry, "mag_mps", 1024, RT_THREAD_PRIORITY_MAX / 2, 20);
        if (mlx90396_thread != RT_NULL)
        {
            rt_thread_startup(mlx90396_thread);

            return 0;
        }
    }
    else if (!strcmp(argv[1], "off"))
    {
        mlx90396_thread = rt_thread_find("mlx90396");

        if (mlx90396_thread != RT_NULL)
        {
            rt_thread_delete(mlx90396_thread);

            return 0;
        }
    }

    return -1;
}

rt_err_t mlx90396_set_sample_freq(int argc, char **argv)
{
    rt_size_t res = RT_EOK;

    sample_freq = atoi(argv[1]);
    rt_kprintf("sample freq = %d\r\n", sample_freq);

    return res;
}

rt_err_t mlx90396_ops_ctrl(int argc, char **argv)
{
    rt_size_t res = RT_EOK;
    rt_device_t dev = RT_NULL;

    rt_uint16_t p = atoi(argv[2]);

    dev = rt_device_find("mag_mps");
    if (dev == RT_NULL)
    {
        LOG_E("Can't find device:%s\n");
        return -RT_ERROR;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            LOG_E("device is already opened!\n");
        }
        else
        {
            LOG_E("open device failed!\n");
            return -RT_ERROR;
        }
    }

    if (rt_device_control(dev, atoi(argv[1]), &p))
    {
        LOG_E("device control set failed, 0x%x 0x%x!\n", atoi(argv[1]), atoi(argv[2]));
        return -RT_ERROR;
    }

    return res;
}

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90396_measurement_onoff, mlx90396 sensor function);
    MSH_CMD_EXPORT(mlx90396_set_sample_freq, mlx90396 sensor function);
    MSH_CMD_EXPORT(mlx90396_ops_ctrl, mlx90396 sensor function);
#endif

