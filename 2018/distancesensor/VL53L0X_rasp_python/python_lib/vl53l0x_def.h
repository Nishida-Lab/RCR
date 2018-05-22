#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

void print_pal_error(VL53L0X_Error Status);
VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev);
void startRanging(int object_number, int mode, uint8_t i2c_address, uint8_t TCA9548A_Device, uint8_t TCA9548A_Address);
int32_t getDistance(int object_number);
void stopRanging(int object_number);
VL53L0X_DEV getDev(int object_number);

