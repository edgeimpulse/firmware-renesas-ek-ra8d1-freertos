/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMON_EVENTS_H_
#define COMMON_EVENTS_H_

#define EVENT_RX_READY                  (1 << 0)
#define EVENT_TX_DONE                   (1 << 1)
#define EVENT_TX_EMPTY                  (1 << 2)
#define CAMERA_READY                    (1 << 3)

#define CAMERA_EVENT_REQUEST            (1 << 0)
#define CAMERA_EVENT_TAKEN              (1 << 1)
#define CAMERA_START_LCD_INFERENCE      (1 << 2)
#define CAMERA_STOP_LCD_INFERENCE       (1 << 3)
#define CAMERA_START_STREAM             (1 << 4)
#define CAMERA_STOP_STREAM              (1 << 5)

#endif /* COMMON_EVENTS_H_ */
