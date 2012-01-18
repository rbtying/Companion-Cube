#ifndef UTILITIES_H_
#define UTILITIES_H_

#ifndef min
#define min(x, y) ((x < y) ? (x) : (y))
#endif

#ifndef max
#define max(x, y) ((x > y) ? (x) : (y))
#endif

#ifndef constrain
#define constrain(x, y, z) (min(max(x, y), z))
#endif

#ifndef scale
#define scale(x, in_min, in_max, out_min, out_max) ((constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#endif

#ifndef normalize
#define normalize(z) atan2(sin(z), cos(z))
#endif

#ifndef abs
#define abs(x) ((x < 0) ? (-x) : (x));
#endif

#ifndef HALF_PI
#define HALF_PI 1.57079633
#endif

#ifndef PI
#define PI 3.14159265
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174532925
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795
#endif

#endif
