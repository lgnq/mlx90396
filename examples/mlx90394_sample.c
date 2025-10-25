/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include <rtthread.h>
#include "mlx90394.h"

/* Default configuration, please change according to the actual situation, support i2c and spi device name */
#define MLX90394_DEVICE_NAME  "i2c2"

/* Test function */
static int mlx90394_test()
{
    struct mlx90394_device *dev;
    int i;

    /* Initialize mlx90394, The parameter is RT_NULL, means auto probing for i2c*/
    dev = mlx90394_init(MLX90394_DEVICE_NAME, RT_NULL);

    if (dev == RT_NULL)
    {
        rt_kprintf("mlx90394 init failed\n");
        return -1;
    }
    rt_kprintf("mlx90394 init succeed\n");

    for (i = 0; i < 5; i++)
    {
        // mpu6xxx_get_gyro(dev, &gyro);

        // rt_kprintf("gyro.x = %3d gyro.y = %3d, gyro.z = %3d\n", gyro.x, gyro.y, gyro.z);

        rt_thread_mdelay(100);
    }

    mlx90394_deinit(dev);

    return 0;
}
MSH_CMD_EXPORT(mlx90394_test, mlx90394 sensor test function);
