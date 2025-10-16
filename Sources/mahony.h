#ifndef SOURCES_MAHONY_H_
#define SOURCES_MAHONY_H_

#define SAMPLE_FREQ_HZ 10.0f

//tuned these
#define KP 0.5f
#define KI 0.005f

//gains
#define TWO_KP (2.0f * KP)
#define TWO_KI (2.0f * KI)

void Mahony_init(void);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az);
void Mahony_getRollPitchYaw(float* roll, float* pitch, float* yaw);

#endif /* SOURCES_MAHONY_H_ */ 