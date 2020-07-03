// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.
#ifndef DEPTH_TYPES_H
#define DEPTH_TYPES_H

#include <astra_core/capi/astra_types.h>
#include <astra/capi/streams/image_types.h>

typedef struct {
    float xzFactor;
    float yzFactor;
    float coeffX;
    float coeffY;
    int resolutionX;
    int resolutionY;
    int halfResX;
    int halfResY;
} astra_conversion_cache_t;

typedef astra_streamconnection_t astra_depthstream_t;
typedef struct _astra_imageframe* astra_depthframe_t;

#endif // DEPTH_TYPES_H
