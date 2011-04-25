#ifndef UTILITIES_H_
#define UTILITIES_H_

#define min(x, y) ((x < y) ? (x) : (y))
#define max(x, y) ((x > y) ? (x) : (y))
#define constrain(x, y, z) (min(max(x, y), z))

#define scale(x, in_min, in_max, out_min, out_max) ((constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define expo(x) ((x) < (0)) ? ((-1) * (x) * (x)) : ((x) * (x))

#define HALF_PI 1.57079633
#define PI 3.14159265

#endif
