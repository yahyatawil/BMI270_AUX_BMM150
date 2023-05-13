# BMI270_AUX_BMM150
Arduino library for BMI270 with BMM150 as an auxiliary sensor. The BMI270_AUX_BMM150 for bmi270 shuttle board is a replacement for [ArduinoBMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) library which is a library that allows you to use the inertial measurement unit (IMU) system available on the Arduino&reg; Nano 33 BLE Sense Rev2 board. The difference between BMI270_AUX_BMM150 and ArduinoBMI270_BMM150 that in the latter, the magnetometer BMM150 is connected to the main I2C bus while in BMI270_AUX_BMM150 the BMM150 is connected as auxiliary sensor. The IMU system is made up of the 3-axis accelerometer and gyroscope [BMI270](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf), and the 3-axis magnetometer [BMM150](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf). The values returned are signed floats.

# Hardware

This library is tested on ESP32 Arduino on [QT Py ESP32 Pico](https://www.adafruit.com/product/5395) connected with [bmi270 shuttle board](https://www.mouser.co.uk/ProductDetail/Bosch-Sensortec/Shuttle-Board-BMI270).  

# Contribution

* PRs are welcome to add support for missing features like: GPIO data ready Interrupt support, FIFO, Features (any motion, no motion, ..etc). Although these features are found in [BMI270-Sensor-API](https://github.com/BoschSensortec/BMI270-Sensor-API) (found in src/utilities folder), but it is not supported in the library class. 
* As the testing is done on ESP32 Arduino at the moment, please report the success on other circuits. Please open an issue to report the success. 
