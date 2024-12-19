/*******************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"

#include "app_bt_gatt_handler.h"

#include "mtb_bmi270.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define POLL_TIMER_IN_MSEC              (1000u)
#define POLL_TIMER_FREQ                 (10000)

/* I2C Clock frequency in Hz */
#define I2C_CLK_FREQ_HZ                 (400000U)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)     (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
/* Structure holding i2c obj */
cyhal_i2c_t i2c_obj;

/* Structure holding bmi270 variables */
mtb_bmi270_t bmi270_obj;

/* Handle to timer object */
TimerHandle_t timer_handle;

typedef enum
{
    ORIENTATION_NULL            = 0,    /* Default orientation state used for initialization purposes */
    ORIENTATION_TOP_EDGE        = 1,    /* Top edge of the board points towards the ceiling */
    ORIENTATION_BOTTOM_EDGE     = 2,    /* Bottom edge of the board points towards the ceiling */
    ORIENTATION_LEFT_EDGE       = 3,    /* Left edge of the board (USB connector side) points towards the ceiling */
    ORIENTATION_RIGHT_EDGE      = 4,    /* Right edge of the board points towards the ceiling */
    ORIENTATION_DISP_UP         = 5,    /* Display faces up (towards the sky/ceiling) */
    ORIENTATION_DISP_DOWN       = 6     /* Display faces down (towards the ground) */
} orientation_t;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/* These static functions are used by the Motion Sensor. These are not
 * available outside this file. See the respective function definitions for
 * more details.
 */
static cy_rslt_t motionsensor_update_orientation(mtb_bmi270_t *dev, orientation_t *orientation);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    int32_t result;

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* Initialize sensor driver here */

    /* I2C Configuration Structure */
    cyhal_i2c_cfg_t i2c_config = { false, 0, I2C_CLK_FREQ_HZ };

    /* Initialize I2C for BMI270 */
    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing i2c\n");
        return -2;
    }

    /* Configure the I2C Interface with the desired clock frequency */
    result = cyhal_i2c_configure(&i2c_obj, &i2c_config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error configuring i2c\n");
        return -3;
    }

    /* Initialize the BMI270 motion sensor*/
    result = mtb_bmi270_init_i2c(&bmi270_obj, &i2c_obj, MTB_BMI270_ADDRESS_SEC);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error initializing bmi270\n");
        return -4;
    }

    /* Configure the BMI270 motion sensor */
    result = mtb_bmi270_config_default(&bmi270_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Error configuring bmi270\n");
        return -5;
    }

    printf("BMI270 Motion Sensor initialized successfully!\n\n");

    return 0;
}

void sensor_task(void *pvParameters)
{
    orientation_t orientation = ORIENTATION_NULL;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Get Board orientation */
        motionsensor_update_orientation(&bmi270_obj, &orientation);

        app_xensiv_sensor_shield_bmi270[0] = orientation;

        if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_bmi270_client_char_config[0]) == 0)
        {
            if(!app_bt_conn_id)
            {
                printf("This device is not connected to a central device\n");
            }
            else
            {
                printf("This device is connected to a central device but\n"
                        "GATT client notifications are not enabled\n");
            }
        }
        else
        {
            wiced_bt_gatt_status_t gatt_status;

            /*
            * Sending notification, set the pv_app_context to NULL, since the
            * data 'app_xensiv_sensor_shield_bmi270' is not to be freed
            */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_XENSIV_SENSOR_SHIELD_BMI270_VALUE,
                                                                 app_xensiv_sensor_shield_bmi270_len,
                                                                 (uint8_t *) app_xensiv_sensor_shield_bmi270,
                                                                 NULL);

            printf("Sent notification status 0x%x\n", gatt_status);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static cy_rslt_t motionsensor_update_orientation(mtb_bmi270_t *dev, orientation_t *orientation)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int abs_x, abs_y, abs_z;

    mtb_bmi270_data_t data;

    /* Read x, y, z components of acceleration */
    result = mtb_bmi270_read(dev, &data);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("read data failed\r\n");
        *orientation = ORIENTATION_NULL;
        return -1;
    }
    abs_x = abs((int) data.sensor_data.acc.x);
    abs_y = abs((int) data.sensor_data.acc.y);
    abs_z = abs((int) data.sensor_data.acc.z);

    printf("x: %d, y: %d, z: %d\r\n", abs_x, abs_y, abs_z);

    if ((abs_z > abs_x) && (abs_z > abs_y))
    {
        if (data.sensor_data.acc.z < 0)
        {
            /* Display faces down (towards the ground) */
            printf("Orientation = ORIENTATION_DISP_DOWN\r\n");
            *orientation = ORIENTATION_DISP_DOWN;
        }
        else
        {
            /* Display faces up (towards the sky/ceiling) */

            printf("Orientation = ORIENTATION_DISP_UP\r\n");
            *orientation = ORIENTATION_DISP_UP;
        }
    }
    /* Y axis (parallel with shorter edge of board) is most aligned with
     * gravity.
     */
    else if ((abs_y > abs_x) && (abs_y > abs_z))
    {
        if (data.sensor_data.acc.y > 0)
        {
            /* Display has an inverted landscape orientation */

            printf("Orientation = ORIENTATION_BOTTOM_EDGE\r\n");
            *orientation = ORIENTATION_BOTTOM_EDGE;
        }
        else
        {
            /* Display has landscape orientation */

            printf("Orientation = ORIENTATION_TOP_EDGE\r\n");
            *orientation = ORIENTATION_TOP_EDGE;
        }
    }
    /* X axis (parallel with longer edge of board) is most aligned with
     * gravity.
     */
    else
    {
        if (data.sensor_data.acc.x < 0)
        {
            /* Display has an inverted portrait orientation */

            printf("Orientation = ORIENTATION_RIGHT_EDGE\r\n");
            *orientation = ORIENTATION_RIGHT_EDGE;
        }
        else
        {
            /* Display has portrait orientation */

            printf("Orientation = ORIENTATION_LEFT_EDGE\r\n");
            *orientation = ORIENTATION_LEFT_EDGE;
        }
    }

    return result;
}
