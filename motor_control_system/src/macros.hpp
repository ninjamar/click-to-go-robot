#pragma once
#include <Arduino.h>

#define MS_PER_S 1000.0f

#define MS_TO_S(ms) ((ms) / MS_PER_S)
#define S_TO_MS(s) ((s) * MS_PER_S)

#define DEG_TO_RADIAN(deg) ((deg) * (PI / 180.0f))
#define RADIAN_TO_DEG(rad) ((rad) * (180.0f / PI))
