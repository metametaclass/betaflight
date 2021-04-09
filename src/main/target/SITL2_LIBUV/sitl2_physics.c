#include "sitl2_physics.h"

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "drivers/accgyro/accgyro_fake.h"
#include "common/maths.h"

#define STD_ACCEL_G 9.80665
#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / STD_ACCEL_G)
#define GYRO_SCALE (16384 / 2000.0f)

//GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors

#define RAND_DEGREE_AMPLITUDE 10

double f_random_one(){
    //range [-1.0, 1.0]
    return (rand()/(0.5*RAND_MAX)) - 1.0;
}

void sitl2_set_random_imu() {
    int16_t x,y,z;
    double xf,yf,zf;

    xf = f_random_one(); //- 0.717 * STD_ACCEL_G
    yf = f_random_one();
    zf = STD_ACCEL_G + f_random_one();

    x = constrain(xf * ACC_SCALE, -32767, 32767);
    y = constrain(yf * ACC_SCALE, -32767, 32767);
    z = constrain(zf * ACC_SCALE, -32767, 32767);

    fakeAccSet(fakeAccDev, x, y, z);

    xf = f_random_one() * RAND_DEGREE_AMPLITUDE;
    yf = f_random_one() * RAND_DEGREE_AMPLITUDE;
    zf = f_random_one() * RAND_DEGREE_AMPLITUDE;

    x = constrain(xf * GYRO_SCALE, -32767, 32767);
    y = constrain(yf * GYRO_SCALE, -32767, 32767);
    z = constrain(zf * GYRO_SCALE, -32767, 32767);

    fakeGyroSet(fakeGyroDev, x, y, z);
}

int sitl2_calc_physics(sitl2_state_t* state){
    sitl2_set_random_imu();

    return 0;
}