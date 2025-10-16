#include <math.h>
#include "mahony.h"

static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_init(void) {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        //normalize accel
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        //error = sum of cross product b/n est and ms dir of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        //integral feedback
        if(TWO_KI > 0.0f) {
            integralFBx += TWO_KI * halfex * (1.0f / SAMPLE_FREQ_HZ);
            integralFBy += TWO_KI * halfey * (1.0f / SAMPLE_FREQ_HZ);
            integralFBz += TWO_KI * halfez * (1.0f / SAMPLE_FREQ_HZ);
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        //proportional feedback
        gx += TWO_KP * halfex;
        gy += TWO_KP * halfey;
        gz += TWO_KP * halfez;
    }

    //integrate RoC of quaternion
    gx *= (0.5f * (1.0f / SAMPLE_FREQ_HZ));
    gy *= (0.5f * (1.0f / SAMPLE_FREQ_HZ));
    gz *= (0.5f * (1.0f / SAMPLE_FREQ_HZ));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    //normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Mahony_getRollPitchYaw(float* roll, float* pitch, float* yaw) {
    float pitch_arg = 2.0f * (q0 * q2 - q3 * q1);

    //clamp pitch arg
    if (pitch_arg > 1.0f) {
        pitch_arg = 1.0f;
    }
    else if (pitch_arg < -1.0f) {
        pitch_arg = -1.0f;
    }

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *pitch = asinf(pitch_arg);
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
} 