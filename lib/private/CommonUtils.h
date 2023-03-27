#ifndef COMMONUTILS_H
#define COMMONUTILS_H

// Angle Conversions
// Convert radians to degrees
#define RAD(x) ((x) / 180.0f * (float) PI)
// Convert degrees to radians
#define DEG(x) ((x) * 180.0f / (float) PI)

// Limit angle to 0 to 360 degrees
#define LIM_ANGLE(angle) (angle > 0 ? fmod(angle, 360) : fmod(angle, 360) + 360)

#endif
