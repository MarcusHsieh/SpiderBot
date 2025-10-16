#ifndef PHOTORESISTOR_H_
#define PHOTORESISTOR_H_

#include "PE_Types.h"
#include "PE_Error.h"

#define PHOTORES_MAX_READING    65535
#define PHOTORES_MIN_READING    0

#define LIGHT_THRESHOLD_BRIGHT  45000  // bright/day
#define LIGHT_THRESHOLD_DIM     20000  // normal
#define LIGHT_THRESHOLD_DARK    5000   // dark/night

// Time of day thresholds
#define DAY_THRESHOLD           25000  // > = DAY
#define NIGHT_THRESHOLD         15000  // < = NIGHT

typedef enum {
    LIGHT_LEVEL_DARK = 0,
    LIGHT_LEVEL_DIM,
    LIGHT_LEVEL_NORMAL,
    LIGHT_LEVEL_BRIGHT
} PhotoRes_LightLevel;

typedef enum {
    TIME_OF_DAY_NIGHT = 0,
    TIME_OF_DAY_DAWN,
    TIME_OF_DAY_DAY,
    TIME_OF_DAY_DUSK
} PhotoRes_TimeOfDay;

uint8_t PhotoRes_Init(void);
uint16_t PhotoRes_ReadRaw(void);
PhotoRes_LightLevel PhotoRes_GetLightLevel(void);
PhotoRes_TimeOfDay PhotoRes_GetTimeOfDay(void);
const char* PhotoRes_GetTimeString(void);
uint8_t PhotoRes_GetLightPercentage(void);
void PhotoRes_PrintDebug(void);
void PhotoRes_StartMeasurement(void);
uint8_t PhotoRes_IsMeasurementComplete(void);

extern volatile uint8_t adc_measurement_complete;

#endif /* PHOTORESISTOR_H_ */ 