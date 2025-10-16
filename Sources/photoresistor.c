#include "photoresistor.h"
#include "AD1.h"
#include "WAIT1.h"
#include "spi_servo_control.h"

static LDD_TDeviceData *adc_device = NULL;
static uint8_t is_initialized = 0;
static uint16_t last_reading = 0;
static LDD_ADC_TSample sample_group[1];

volatile uint8_t adc_measurement_complete = 0;

uint8_t PhotoRes_Init(void) {
    adc_device = AD1_DeviceData;
    if (adc_device == NULL) {
        return 1;
    }

    sample_group[0].ChannelIdx = 0;

    LDD_TError error = AD1_CreateSampleGroup(adc_device, sample_group, 1);
    if (error != ERR_OK) {
        return 2;
    }
    
    is_initialized = 1;
    return 0;
}

void PhotoRes_StartMeasurement(void) {
    if (!is_initialized || adc_device == NULL) {
        return;
    }
    
    adc_measurement_complete = 0;
    AD1_StartSingleMeasurement(adc_device);
}

uint8_t PhotoRes_IsMeasurementComplete(void) {
    return adc_measurement_complete;
}

uint16_t PhotoRes_ReadRaw(void) {
    if (!is_initialized || adc_device == NULL) {
        return 0;
    }
    
    uint16_t result = 0;
    LDD_TError error;
    
    PhotoRes_StartMeasurement();
    
    uint16_t timeout = 1000;
    while (!PhotoRes_IsMeasurementComplete() && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (timeout == 0) {
        return last_reading;
    }

    error = AD1_GetMeasuredValues(adc_device, (LDD_TData*)&result); //result!!
    if (error == ERR_OK) {
        last_reading = result;
    }
    
    return last_reading;
}

PhotoRes_LightLevel PhotoRes_GetLightLevel(void) {
    uint16_t raw_value = PhotoRes_ReadRaw();
    
    if (raw_value >= LIGHT_THRESHOLD_BRIGHT) {
        return LIGHT_LEVEL_BRIGHT;
    } else if (raw_value >= LIGHT_THRESHOLD_DIM) {
        return LIGHT_LEVEL_NORMAL;
    } else if (raw_value >= LIGHT_THRESHOLD_DARK) {
        return LIGHT_LEVEL_DIM;
    } else {
        return LIGHT_LEVEL_DARK;
    }
}

PhotoRes_TimeOfDay PhotoRes_GetTimeOfDay(void) {
    uint16_t raw_value = PhotoRes_ReadRaw();
    
    if (raw_value >= DAY_THRESHOLD) {
        return TIME_OF_DAY_DAY;
    } else if (raw_value >= NIGHT_THRESHOLD) {
        return TIME_OF_DAY_DAWN;
    } else {
        return TIME_OF_DAY_NIGHT;
    }
}

const char* PhotoRes_GetTimeString(void) {
    PhotoRes_TimeOfDay time = PhotoRes_GetTimeOfDay();
    
    switch (time) {
        case TIME_OF_DAY_DAY:
            return "day";
        case TIME_OF_DAY_DAWN:
            return "dawn";
        case TIME_OF_DAY_DUSK:
            return "dusk";
        case TIME_OF_DAY_NIGHT:
        default:
            return "night";
    }
}

uint8_t PhotoRes_GetLightPercentage(void) {
    uint16_t raw_value = PhotoRes_ReadRaw();
    
    uint32_t percentage = ((uint32_t)raw_value * 100) / PHOTORES_MAX_READING;
    
    if (percentage > 100) {
        percentage = 100;
    }
    
    return (uint8_t)percentage;
}

void PhotoRes_PrintDebug(void) {
    uint16_t raw = PhotoRes_ReadRaw();
    uint8_t percentage = PhotoRes_GetLightPercentage();
    PhotoRes_LightLevel level = PhotoRes_GetLightLevel();
    const char* time_str = PhotoRes_GetTimeString();
    
    SpiServo_DebugPuts("Light Sensor - Raw: ");
    SpiServo_DebugPrintDec(raw);
    SpiServo_DebugPuts(", Percent: ");
    SpiServo_DebugPrintDec(percentage);
    SpiServo_DebugPuts("%, Level: ");
    
    switch (level) {
        case LIGHT_LEVEL_DARK:
            SpiServo_DebugPuts("DARK");
            break;
        case LIGHT_LEVEL_DIM:
            SpiServo_DebugPuts("DIM");
            break;
        case LIGHT_LEVEL_NORMAL:
            SpiServo_DebugPuts("NORMAL");
            break;
        case LIGHT_LEVEL_BRIGHT:
            SpiServo_DebugPuts("BRIGHT");
            break;
    }
    
    SpiServo_DebugPuts(", Time: ");
    SpiServo_DebugPuts(time_str);
    SpiServo_DebugPuts("\r\n");
} 