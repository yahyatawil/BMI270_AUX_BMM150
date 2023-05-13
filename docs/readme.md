# ArduinoBMI_270_BMM150 library

The BMI270_AUX_BMM150 for bmi270 shuttle board is a replacement for [ArduinoBMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) library which is a library that allows you to use the inertial measurement unit (IMU) system available on the Arduino&reg; Nano 33 BLE Sense Rev2 board. The difference between BMI270_AUX_BMM150 and ArduinoBMI270_BMM150 that in the latter, the magnetometer BMM150 is connected to the main I2C bus while in BMI270_AUX_BMM150 the BMM150 is connected as auxiliary sensor. The IMU system is made up of the 3-axis accelerometer and gyroscope [BMI270](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf), and the 3-axis magnetometer [BMM150](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf). The values returned are signed floats.

To use this library:

```
#include "BMI270_AUX_BMM150.h"
```

The BMI270_AUX_BMM150 library takes care of the sensor initialization and sets its values as follows:

- Accelerometer range is set at ±4 g with a resolution of 0.122 mg.
- Gyroscope range is set at ±2000 dps with a resolution of 70 mdps.
- Magnetometer range is set at ±1300 uT with a resolution of 0.3 uT.
- Accelerometer and gyrospcope output data rate is fixed at 99.84 Hz.
- Magnetometer output data rate is fixed at 10 Hz.
