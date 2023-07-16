/*
  This file is part of the BMI270_AUX_BMM150 library.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef _BMI2_BMM1_H_
#define _BMI2_BMM1_H_

#include <Arduino.h>
#include <Wire.h>
#include "utilities/bmi270.h"
#include "utilities/bmm150.h"
#include "utilities/bmi2.h"
#include "utilities/bmi2_defs.h"

struct dev_info {
  TwoWire* _wire;
  uint8_t dev_addr;
  struct bmi2_dev * bmi2;
  struct bmm150_dev * bmm1;
};
    
typedef struct bmi2_sens_data imu_data_t;
typedef struct bmm150_mag_data mag_data_t;

class BMI2_BMM1_Class {
  public:
    BMI2_BMM1_Class(TwoWire& wire = Wire);
    ~BMI2_BMM1_Class() {}

    int begin(Stream * debug_port=&Serial );
    void debug(Stream& stream);
    void end();

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in G (earth gravity).
     int readGyroscope(float& x, float& y, float& z);
    int readMagneticField(float& x, float& y, float& z);
    
    int magneticFieldSampleRate();
    
    int accelerationAvailable(); // Number of samples in the FIFO.
    int magneticFieldAvailable() ;
    
    float accelerationSampleRate(); // Sampling rate of the sensor.

    int readGyroAccel(imu_data_t & imu_sensor_data, bool raw = false); // Results are in degrees/second.
    int readAuxMag(mag_data_t & mag_sensor_data);
    int gyroscopeAvailable(); // Number of samples in the FIFO.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.

  private:

    static int8_t i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
    static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
    static void delay_usec(uint32_t period_us, void *intf_ptr);
    void print_rslt(int8_t rslt);
    int8_t bmi2_accel_set_config(struct bmi2_dev *bmi2_dev);    
    int8_t bmi2_gyro_set_config(struct bmi2_dev *bmi2_dev);
    int8_t bmi2_mag_set_config(struct bmm150_dev *dev);
    
  private:
    TwoWire* _wire;
    struct dev_info accel_gyro_dev_info;
    struct dev_info mag_dev_info;
    struct bmi2_dev bmi2;
    struct bmm150_dev bmm1;
    uint16_t _int_status;
    Stream * _debug = NULL;
};

extern BMI2_BMM1_Class IMU_BMI270_BMM150;
#undef IMU
#define IMU IMU_BMI270_BMM150

#endif
