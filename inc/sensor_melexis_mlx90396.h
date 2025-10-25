/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __SENSOR_MELEXIS_MLX90396_H__
#define __SENSOR_MELEXIS_MLX90396_H__

#include "sensor.h"
#include "mlx90396.h"

enum
{
    RT_SENSOR_CTRL_USER_CMD_RESET = 0x101,
    RT_SENSOR_CTRL_USER_CMD_INFO,
    RT_SENSOR_CTRL_USER_CMD_XONOFF,
    RT_SENSOR_CTRL_USER_CMD_YONOFF,
    RT_SENSOR_CTRL_USER_CMD_ZONOFF,
    RT_SENSOR_CTRL_USER_CMD_TONOFF,
    RT_SENSOR_CTRL_USER_CMD_SET_MODE,
    RT_SENSOR_CTRL_USER_CMD_SET_RANGE,
    RT_SENSOR_CTRL_USER_CMD_SET_FILT_XY,
    RT_SENSOR_CTRL_USER_CMD_SET_FILT_Z,
    RT_SENSOR_CTRL_USER_CMD_SET_FILT_T,
    RT_SENSOR_CTRL_USER_CMD_SET_WOC_MODE,
    RT_SENSOR_CTRL_USER_CMD_SET_OSR_HALL,
    RT_SENSOR_CTRL_USER_CMD_SET_OSR_TEMP,
};

int rt_hw_mlx90396_init(const char *name, struct rt_sensor_config *cfg);

#endif
