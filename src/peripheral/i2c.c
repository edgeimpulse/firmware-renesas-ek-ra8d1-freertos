/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Include ----------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "event_groups.h"

#include "i2c.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Global Variables -------------------------------------------------------- */
#define RESET_VALUE                                 0
#define I2C_TIMEOUT_TICKS                           (250)  /* Timeout for the I"C transmission and Reception*/


EventGroupHandle_t i2c_riic0_event_handle = {0};
StaticEventGroup_t i2c_riic1_event_buf = {0};

/* Global Variables -------------------------------------------------------- */
static void g_comms_i2c_bus1_quick_setup(void);
static void g_comms_i2c_bus1_quick_shutdown(void);

/* Public functions -------------------------------------------------------- */
/**
 * @brief Init I2C peripheral
 * @return
 *
 * @note default speed is 100kHz
 */
int ei_i2c_init(void)
{
    fsp_err_t               err     = FSP_SUCCESS;
    i2c_master_status_t     status;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus1_extended_cfg.p_driver_instance;

    /* opening IIC master module */
    g_comms_i2c_bus1_quick_setup();

    R_IIC_MASTER_StatusGet(p_driver_instance->p_ctrl, &status);

    if (status.open != true) {
        err = FSP_ERR_NOT_OPEN;
    }

    i2c_riic0_event_handle = xEventGroupCreateStatic( &i2c_riic1_event_buf );

    return (int)err;
}

/**
 *
 * @return
 */
int ei_i2c_deinit(void)
{
    fsp_err_t err     = FSP_SUCCESS;
    i2c_master_status_t     status;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus1_extended_cfg.p_driver_instance;

    g_comms_i2c_bus1_quick_shutdown();

    R_IIC_MASTER_StatusGet(p_driver_instance->p_ctrl, &status);

    if (status.open != false){
        err = FSP_ERR_IN_USE;
    }

    return (int)err;
}

/**
 *
 * @param address
 * @param data
 * @param bytes
 * @return
 */
int ei_i2c_write(rm_comms_ctrl_t * const p_api_ctrl, uint8_t*data, uint16_t bytes)
{
    fsp_err_t err     = FSP_SUCCESS;
    EventBits_t events = {0};

    err = RM_COMMS_I2C_Write(p_api_ctrl, data, bytes);  /* address to read */
    /* Wait for TX end */
    events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                  (1 << RM_COMMS_EVENT_OPERATION_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  I2C_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << RM_COMMS_EVENT_OPERATION_COMPLETE)) != 0), FSP_ERR_ABORTED );

    return (int)err;
}

/**
 *
 * @param address
 * @param read_cmd
 * @param read_data
 * @param bytes
 * @return
 */
int ei_i2c_read_word_command(rm_comms_ctrl_t * const p_api_ctrl, uint16_t read_cmd, uint8_t* read_data, uint16_t bytes)
{
    uint8_t tx_buff[2] = {0};
    fsp_err_t err     = FSP_SUCCESS;
    EventBits_t events = {0};

    tx_buff[0] = (uint8_t)((read_cmd & 0xFF00) >> 8);
    tx_buff[1] = (uint8_t)(read_cmd & 0x00FF);
    err = RM_COMMS_I2C_Write(p_api_ctrl, tx_buff, 2u);  /* address to read */

    /* Wait for TX end */
    events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                  (1 << RM_COMMS_EVENT_OPERATION_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  I2C_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << RM_COMMS_EVENT_OPERATION_COMPLETE)) != 0), FSP_ERR_ABORTED );

    if (FSP_SUCCESS == err) {
        err  = RM_COMMS_I2C_Read(p_api_ctrl, read_data, bytes);

        /* Wait for RX end */
        events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                      (1 << RM_COMMS_EVENT_OPERATION_COMPLETE),
                                      pdTRUE,
                                      pdFALSE,
                                      I2C_TIMEOUT_TICKS );
        FSP_ERROR_RETURN( ((events & (1 << RM_COMMS_EVENT_OPERATION_COMPLETE)) != 0), FSP_ERR_ABORTED );

        err = FSP_SUCCESS;
    }
    /* handle error */
    else {

    }

    return (int)err;
}

/**
 *
 * @param address
 * @param data
 * @param bytes
 * @return
 */
int ei_i2c_read_byte_command(rm_comms_ctrl_t * const p_api_ctrl, uint8_t read_cmd, uint8_t* read_data, uint16_t bytes)
{
    uint8_t tx_buff[2] = {0};
    fsp_err_t err     = FSP_SUCCESS;
    EventBits_t events = {0};

    tx_buff[0] = read_cmd;
    err = RM_COMMS_I2C_Write(p_api_ctrl, tx_buff, 1u);  /* address to read */
    /* Wait for TX end */
    events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                  (1 << RM_COMMS_EVENT_OPERATION_COMPLETE),
                                  pdTRUE,
                                  pdFALSE,
                                  I2C_TIMEOUT_TICKS );

    FSP_ERROR_RETURN( ((events & (1 << RM_COMMS_EVENT_OPERATION_COMPLETE)) != 0), FSP_ERR_ABORTED );

    if (FSP_SUCCESS == err) {

        err  = RM_COMMS_I2C_Read(p_api_ctrl, read_data, bytes);
        FSP_ERROR_RETURN( (err == FSP_SUCCESS), err );
        /* Wait for RX end */
        events = xEventGroupWaitBits( i2c_riic0_event_handle,
                                      (1 << RM_COMMS_EVENT_OPERATION_COMPLETE),
                                      pdTRUE,
                                      pdFALSE,
                                      I2C_TIMEOUT_TICKS );
        FSP_ERROR_RETURN( ((events & (1 << RM_COMMS_EVENT_OPERATION_COMPLETE)) != 0), FSP_ERR_ABORTED );

        err = FSP_SUCCESS;
    }

    return (int)err;
}

/**
 *
 * @param p_args
 */
void comms_i2c_callback(rm_comms_callback_args_t *p_args)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    if (NULL != p_args) {


        xEventGroupSetBitsFromISR( i2c_riic0_event_handle,
                                   (1 << p_args->event),
                                   &pxHigherPriorityTaskWoken );
    }

    portYIELD_FROM_ISR( pxHigherPriorityTaskWoken );
}
/* Private functions -------------------------------------------------------- */

/* Quick setup for g_comms_i2c_bus0. */
static void g_comms_i2c_bus1_quick_setup(void)
{
    fsp_err_t err;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus1_extended_cfg.p_driver_instance;

    /* Open I2C driver, this must be done before calling any COMMS API */
    err = p_driver_instance->p_api->open(p_driver_instance->p_ctrl, p_driver_instance->p_cfg);
    assert(FSP_SUCCESS == err);

#if BSP_CFG_RTOS
    /* Create a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_semaphore_create(g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_handle,
                            g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_name,
                            (ULONG) 0);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_handle)
            = xSemaphoreCreateCountingStatic((UBaseType_t) 1, (UBaseType_t) 0, g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_memory);
#endif
    }

    /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_mutex_create(g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_handle,
                        g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_name,
                        TX_INHERIT);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_handle)
            = xSemaphoreCreateRecursiveMutexStatic(g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_memory);
#endif
    }
#endif
}

/* Quick shutdown for g_comms_i2c_bus0. */
static void g_comms_i2c_bus1_quick_shutdown(void)
{
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus1_extended_cfg.p_driver_instance;

    /* Close I2C driver */
    p_driver_instance->p_api->close(p_driver_instance->p_ctrl);

#if BSP_CFG_RTOS
    /* Delete a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_semaphore_delete(g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_handle);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        vSemaphoreDelete(*(g_comms_i2c_bus1_extended_cfg.p_blocking_semaphore->p_semaphore_handle));
#endif
    }

    /* Delete a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_mutex_delete(g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_handle);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        vSemaphoreDelete(*(g_comms_i2c_bus1_extended_cfg.p_bus_recursive_mutex->p_mutex_handle));
#endif
    }
#endif
}
