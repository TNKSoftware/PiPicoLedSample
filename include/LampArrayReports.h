// Based on ArduinoHidForWindows
// Copyright (c) Microsoft Corporation.  All rights reserved.
// Licensed under the MIT License.

#pragma once

#define LAMP_ARRAY_ATTRIBUTES 1
struct __attribute__ ((__packed__)) LampArrayAttributesReport
{
    uint16_t LampCount;
    uint32_t BoundingBoxWidthInMicrometers;
    uint32_t BoundingBoxHeightInMicrometers;
    uint32_t BoundingBoxDepthInMicrometers;
    uint32_t LampArrayKind;
    uint32_t MinUpdateIntervalInMicroseconds;
};

#define LAMP_ATTRIBUTES_REQUEST 2
struct __attribute__ ((__packed__)) LampAttributesRequestReport
{
    uint16_t LampId;
};

#define LAMP_ATTRIBUTES_RESPONSE 3
struct __attribute__ ((__packed__)) LampAttributesResponseReport
{
    LampAttributes Attributes;
};

#define LAMP_MULTI_UPDATE 4
#define LAMP_MULTI_UPDATE_LAMP_COUNT 8
struct __attribute__ ((__packed__)) LampMultiUpdateReport
{
    uint8_t LampCount;
    uint8_t LampUpdateFlags;
    uint16_t LampIds[LAMP_MULTI_UPDATE_LAMP_COUNT];
    LampArrayColor UpdateColors[LAMP_MULTI_UPDATE_LAMP_COUNT];
};

#define LAMP_RANGE_UPDATE 5
struct __attribute__ ((__packed__)) LampRangeUpdateReport
{
    uint8_t LampUpdateFlags;
    uint16_t LampIdStart;
    uint16_t LampIdEnd;
    LampArrayColor UpdateColor;
};

#define LAMP_ARRAY_CONTROL 6
struct __attribute__ ((__packed__)) LampArrayControlReport
{
    uint8_t AutonomousMode;
};