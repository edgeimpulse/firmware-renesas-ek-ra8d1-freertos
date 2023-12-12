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
/** Include ----------------------------------------------------------------- */
#include "ei_device_renesas_ek_ra8d1.h"
#include <string>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#if (USE_UART == 1)
#include <peripheral/uart.h>
#endif
/* Constants --------------------------------------------------------------- */

const ei_device_data_output_baudrate_t ei_dev_default_data_output_baudrate = {
    "115200",
    115200,
};
#if (USE_UART == 1)
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    "921600",
    921600,
};
#else
const ei_device_data_output_baudrate_t ei_dev_max_data_output_baudrate = {
    "36846400",
    36846400u,
};
#endif

/******
 *
 * @brief EdgeImpulse Device structure and information
 *
 ******/

/**
 *
 * @param code_flash
 * @param data_flash_to_set
 */
EiDeviceRenesasEKRA8D1::EiDeviceRenesasEKRA8D1(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set)
{
    EiDeviceInfo::memory = code_flash;
    EiDeviceRenesasEKRA8D1::data_flash = data_flash_to_set;

    init_device_id();
    load_config();

    device_type = "RENESAS_EK_RA8M1";
    state = eiStateIdle;
    using_max_baud = false;
    camera_is_init = false;

    /* Init camera instance */
    cam = static_cast<EiCameraArduCam*>(EiCamera::get_camera());
    camera_is_present = cam->initialize_camera();
}

/**
 *
 */
EiDeviceRenesasEKRA8D1::~EiDeviceRenesasEKRA8D1()
{

}

/**
 *
 * @return
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    /* Initializing EdgeImpulse classes here in order for
     * Flash memory to be initialized before mainloop start
     */
    static EiFlashMemory code_memory(e_flash_code, sizeof(EiConfig));
    static EiFlashMemory data_memory(e_flash_data, 0);                  /* code flash doesn't store config !*/
    static EiDeviceRenesasEKRA8D1 dev(&code_memory, &data_memory);

    return &dev;
}

/**
 *
 */
void EiDeviceRenesasEKRA8D1::init_device_id(void)
{
    const bsp_unique_id_t *pdev_id;
    char temp[20];

    pdev_id = R_BSP_UniqueIdGet();

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x:%02x",
            pdev_id->unique_id_bytes[5],
            pdev_id->unique_id_bytes[4],
            pdev_id->unique_id_bytes[3],
            pdev_id->unique_id_bytes[2],
            pdev_id->unique_id_bytes[1],
            pdev_id->unique_id_bytes[0]);

    device_id = std::string(temp);
}

/**
 *
 */
void EiDeviceRenesasEKRA8D1::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

/**
 *
 * @return
 */
uint32_t EiDeviceRenesasEKRA8D1::get_data_output_baudrate(void)
{
    return ei_dev_max_data_output_baudrate.val;
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceRenesasEKRA8D1::set_max_data_output_baudrate()
{
#if (USE_UART == 1)
    if (uart_set_baud(true)  == FSP_SUCCESS) {
        using_max_baud = true;
    }
#else
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceRenesasEKRA8D1::set_default_data_output_baudrate()
{
#if (USE_UART == 1)
    if (uart_set_baud(false)  == FSP_SUCCESS) {
        using_max_baud = false;
    }
#else
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

/**
 *
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return
 */
bool EiDeviceRenesasEKRA8D1::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    bool result = false;

    /* TODO-MODIFY Add target specific code to enable a sampling timer */

    return result;
}

bool EiDeviceRenesasEKRA8D1::stop_sample_thread(void)
{
    /* TODO-MODIFY Add target specific code to stop the sampling timer */
    this->set_state(eiStateIdle);

    return true;
}

/**
 *
 * @param state
 */
void EiDeviceRenesasEKRA8D1::set_state(EiState state)
{
    this->state = state;

    /* TODO-OPTIONAL Use when LED GPIO is implemented */
    switch(state) {
        case eiStateErasingFlash:
        case eiStateSampling:
        case eiStateUploading:
        case eiStateFinished:
        case eiStateIdle:
        default:
            break;
    }
}

/**
 *
 * @return
 */
EiState EiDeviceRenesasEKRA8D1::get_state(void)
{
    return this->state;
}

/**
 *
 * @return
 */
EiSnapshotProperties EiDeviceRenesasEKRA8D1::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;
    uint8_t res_num = 0;

    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = 0,
        .resolutions = res
    };

    if(this->cam->is_camera_present() == true) {
        this->cam->get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }

    return props;
}

/**
 *
 */
void EiDeviceRenesasEKRA8D1::sample_thread(void)
{

}

/**
 *
 * @param sensor_list
 * @param sensor_list_size
 * @return
 */
bool EiDeviceRenesasEKRA8D1::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list = sensors;
    *sensor_list_size = 0;
    return true;
}

/**
 *
 */
void EiDeviceRenesasEKRA8D1::init_camera(void)
{
    if (camera_is_init == false) {
        camera_is_present = cam->initialize_camera();
        camera_is_init = true;
    }

}
