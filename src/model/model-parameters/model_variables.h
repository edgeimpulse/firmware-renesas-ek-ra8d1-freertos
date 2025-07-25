/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Generated by Edge Impulse and licensed under the applicable Edge Impulse
 * Terms of Service. Community and Professional Terms of Service
 * (https://edgeimpulse.com/legal/terms-of-service) or Enterprise Terms of
 * Service (https://edgeimpulse.com/legal/enterprise-terms-of-service),
 * according to your product plan subscription (the “License”).
 *
 * This software, documentation and other associated files (collectively referred
 * to as the “Software”) is a single SDK variation generated by the Edge Impulse
 * platform and requires an active paid Edge Impulse subscription to use this
 * Software for any purpose.
 *
 * You may NOT use this Software unless you have an active Edge Impulse subscription
 * that meets the eligibility requirements for the applicable License, subject to
 * your full and continued compliance with the terms and conditions of the License,
 * including without limitation any usage restrictions under the applicable License.
 *
 * If you do not have an active Edge Impulse product plan subscription, or if use
 * of this Software exceeds the usage limitations of your Edge Impulse product plan
 * subscription, you are not permitted to use this Software and must immediately
 * delete and erase all copies of this Software within your control or possession.
 * Edge Impulse reserves all rights and remedies available to enforce its rights.
 *
 * Unless required by applicable law or agreed to in writing, the Software is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language governing
 * permissions, disclaimers and limitations under the License.
 */

#ifndef _EI_CLASSIFIER_MODEL_VARIABLES_H_
#define _EI_CLASSIFIER_MODEL_VARIABLES_H_

/**
 * @file
 *  Auto-generated complete impulse definitions. The ei_impulse_handle_t should
 *  be passed to ei_run_classifier() function to use this specific impulse.
 *  This file should only be included in ei_run_classifier.h file.
 */

#include <stdint.h>
#include "model_metadata.h"

#include "tflite-model/tflite_learn_33_compiled.h"
#include "edge-impulse-sdk/classifier/ei_model_types.h"
#include "edge-impulse-sdk/classifier/inferencing_engines/engines.h"
#include "edge-impulse-sdk/classifier/postprocessing/ei_postprocessing_common.h"

const char* ei_classifier_inferencing_categories[] = { "face" };

uint8_t ei_dsp_config_32_axes[] = { 0 };
const uint32_t ei_dsp_config_32_axes_size = 1;
ei_dsp_config_image_t ei_dsp_config_32 = {
    32, // uint32_t blockId
    1, // int implementationVersion
    1, // int length of axes
    NULL, // named axes
    0, // size of the named axes array
    "RGB" // select channels
};

const uint8_t ei_dsp_blocks_size = 1;
ei_model_dsp_t ei_dsp_blocks[ei_dsp_blocks_size] = {
    { // DSP block 32
        32,
        27648, // output size
        &extract_image_features, // DSP function pointer
        (void*)&ei_dsp_config_32, // pointer to config struct
        ei_dsp_config_32_axes, // array of offsets into the input stream, one for each axis
        ei_dsp_config_32_axes_size, // number of axes
        1, // version
        nullptr, // factory function
    }
};
const ei_config_tflite_eon_graph_t ei_config_tflite_graph_33 = {
    .implementation_version = 1,
    .model_init = &tflite_learn_33_init,
    .model_invoke = &tflite_learn_33_invoke,
    .model_reset = &tflite_learn_33_reset,
    .model_input = &tflite_learn_33_input,
    .model_output = &tflite_learn_33_output,
};

const uint8_t ei_output_tensors_indices_33[1] = { 0 };
const uint8_t ei_output_tensors_size_33 = 1;
const ei_learning_block_config_tflite_graph_t ei_learning_block_config_33 = {
    .implementation_version = 1,
    .block_id = 33,
    .output_tensors_indices = ei_output_tensors_indices_33,
    .output_tensors_size = ei_output_tensors_size_33,
    .quantized = 1,
    .compiled = 1,
    .graph_config = (void*)&ei_config_tflite_graph_33
};

const uint8_t ei_learning_blocks_size = 1;
const uint32_t ei_learning_block_33_inputs[1] = { 32 };
const uint8_t ei_learning_block_33_inputs_size = 1;
const ei_learning_block_t ei_learning_blocks[ei_learning_blocks_size] = {
    {
        33,
        &run_nn_inference,
        (void*)&ei_learning_block_config_33,
        EI_CLASSIFIER_IMAGE_SCALING_NONE,
        ei_learning_block_33_inputs,
        ei_learning_block_33_inputs_size,
    },
};

const ei_fill_result_fomo_i8_config_t ei_fill_result_fomo_i8_config_33 = {
    .threshold = 0.55,
    .out_width = 12,
    .out_height = 12,
    .object_detection_count = 10,
    .zero_point = -128,
    .scale = 0.00390625
};
const size_t ei_postprocessing_blocks_size = 1;
const ei_postprocessing_block_t ei_postprocessing_blocks[ei_postprocessing_blocks_size] = {
    {
        .block_id = 34,
        .type = EI_CLASSIFIER_MODE_OBJECT_DETECTION,
        .init_fn = NULL,
        .deinit_fn = NULL,
        .postprocess_fn = &process_fomo_i8,
        .display_fn = NULL,
        .config = (void*)&ei_fill_result_fomo_i8_config_33,
        .input_block_id = 33,
    },
};

const ei_impulse_t impulse_49_0 = {
    .project_id = 49,
    .project_owner = "Edge Impulse Profiling",
    .project_name = "Demo: Constrained Object Detection",
    .impulse_id = 1,
    .impulse_name = "Impulse #1",
    .deploy_version = 6,

    .nn_input_frame_size = 27648,
    .raw_sample_count = 9216,
    .raw_samples_per_frame = 1,
    .dsp_input_frame_size = 9216 * 1,
    .input_width = 96,
    .input_height = 96,
    .input_frames = 1,
    .interval_ms = 1,
    .frequency = 0,

    .dsp_blocks_size = ei_dsp_blocks_size,
    .dsp_blocks = ei_dsp_blocks,

    .learning_blocks_size = ei_learning_blocks_size,
    .learning_blocks = ei_learning_blocks,

    .postprocessing_blocks_size = 1,
    .postprocessing_blocks = ei_postprocessing_blocks,

    .inferencing_engine = EI_CLASSIFIER_TFLITE,

    .sensor = EI_CLASSIFIER_SENSOR_CAMERA,
    .fusion_string = "image",
    .slice_size = (9216/4),
    .slices_per_model_window = 4,

    .has_anomaly = EI_ANOMALY_TYPE_UNKNOWN,
    .label_count = 1,
    .categories = ei_classifier_inferencing_categories
};

ei_impulse_handle_t impulse_handle_49_0 = ei_impulse_handle_t( &impulse_49_0 );
ei_impulse_handle_t& ei_default_impulse = impulse_handle_49_0;

#endif // _EI_CLASSIFIER_MODEL_VARIABLES_H_
