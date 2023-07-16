/*
  This file is part of the BMI270_AUX_BMM150 library.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "Arduino.h"
#include "Wire.h"
#include "BMI2_BMM1.h"

BMI2_BMM1_Class::BMI2_BMM1_Class(TwoWire& wire)
{
        _wire = &wire;
}

void BMI2_BMM1_Class::debug(Stream& stream){
_debug = &stream; 
}

int BMI2_BMM1_Class::begin(Stream * debug_port) {

   /* Variable to define result */
  int8_t rslt;
  
  _debug = debug_port;
  
  /* To initialize the hal function */
  _wire->begin();

   
  accel_gyro_dev_info._wire = _wire;
  accel_gyro_dev_info.dev_addr = BMI2_I2C_PRIM_ADDR;
  accel_gyro_dev_info.bmi2 = &bmi2;
  accel_gyro_dev_info.bmm1 = &bmm1;
  
  bmi2.intf_ptr = &accel_gyro_dev_info;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.read = i2c_bus_read;
  bmi2.write = i2c_bus_write;
  bmi2.read_write_len = 32;
  bmi2.delay_us = delay_usec;

          
  /* Config file pointer should be assigned to NULL, so that default file address is assigned in bmi270_init */
  bmi2.config_file_ptr = NULL;

  /* Initialize bmi270 */
  rslt = bmi270_init(&bmi2);

//  _debug->println("bmi270_init done");

  rslt = bmi2_accel_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    _debug->println("Accel data Ready : X Y Z ");
  }

  rslt = bmi2_gyro_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    _debug->println("Gyro data Ready : X Y Z ");
  }
  else{
    _debug->println("BMI270 Init failed /r/n");
    while(1);
  }

    /* To enable the i2c interface settings for bmm150. */
    uint8_t aux_bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    bmm1.chip_id = BMM150_DEFAULT_I2C_ADDRESS;
    bmm1.intf_ptr = &accel_gyro_dev_info;
    bmm1.read = aux_i2c_read;
    bmm1.write = aux_i2c_write;
    bmm1.delay_us = (bmi2_delay_fptr_t) & delay_usec;

    /* As per datasheet, aux interface with bmi270 will support only for I2C */
   bmm1.intf = BMM150_I2C_INTF;
    
  rslt = bmi2_mag_set_config(&bmm1);
  
  if (rslt == BMI2_OK)
  {
    _debug->println("Mag data Ready : X Y Z ");
  }
  else{
    _debug->println("BMM150 Init failed /r/n");
    while(1);
  }

  return 1;
}


// default range is +-4G, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_G   (8192.0f)

void BMI2_BMM1_Class::delay_usec(uint32_t period_us, void *intf_ptr)
{
  delayMicroseconds(period_us);
}

/*! This API is used to perform I2C read operation with sensor */
int8_t BMI2_BMM1_Class::i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  /* dev_addr: I2C device address.
    reg_addr: Starting address for writing the data.
    reg_data: Data to be written.
    count: Number of bytes to write */
  // Begin I2C communication with provided I2C address
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();

  if (returned)
  {
    /*
      case 1:Data too long to fit in transmit buffer
          break;
      case 2:received NACK on transmit of address.
          break;
      case 3:received NACK on transmit of data."
          break;
      case 4:Unspecified error.
          break;
      default:Unexpected Wire.endTransmission() return code:
    */
    return returned;
  }

  // Requests the required number of bytes from the sensor
  dev_info->_wire->requestFrom((int)dev_id, (int)length);

  uint16_t i;
  // Reads the requested number of bytes into the provided array
  for (i = 0; (i < length) && dev_info->_wire->available(); i++)
  {
    reg_data[i] = dev_info->_wire->read(); // This is for the modern Wire library
  }
  
  // This must return 0 on success, any other value will be interpreted as a communication failure.
  return rslt;

}

/*! This API is used to perform I2C write operations with sensor */
int8_t BMI2_BMM1_Class::i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  struct dev_info* dev_info = (struct dev_info*)intf_ptr;
  uint8_t dev_id = dev_info->dev_addr;
  
  /*  dev_addr: I2C device address.
    reg_addr: Starting address for reading the data.
    reg_data: Buffer to take up the read data.
    count: Number of bytes to read. */
  // Begin I2C communication with provided I2C address
  dev_info->_wire->beginTransmission(dev_id);
  dev_info->_wire->write(reg_addr);

  uint16_t i;
  // Writes the requested number of bytes from the provided array
  for (i = 0; i < length; i++)
  {
    dev_info->_wire->write(reg_data[i]); // This is for the modern Wire library
  }
  // Done writting, end the transmission
  rslt = dev_info->_wire->endTransmission();
  /*
      case 1:Data too long to fit in transmit buffer
      case 2:received NACK on transmit of address.
      case 3:received NACK on transmit of data.
      case 4:Unspecified error.
      default:Unexpected Wire.endTransmission() return code:
  */
  // This must return 0 on sucess, any other value will be interpretted as a communication failure.
  return rslt;
  
}

int8_t BMI2_BMM1_Class::aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;
    struct dev_info* dev_info = (struct dev_info*)intf_ptr;
    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, length, dev_info->bmi2);

    return rslt;
}

/*!
 * @brief This function writes the data to auxiliary sensor in data mode.
 */
int8_t BMI2_BMM1_Class::aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;
    struct dev_info* dev_info = (struct dev_info*)intf_ptr;
    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, length, dev_info->bmi2);

    return rslt;
}

int8_t BMI2_BMM1_Class::bmi2_accel_set_config(struct bmi2_dev *bmi2_dev)
{
  /* Variable to define result */
  int8_t rslt;

  /* Initialize interrupts for gyroscope */
  uint8_t sens_int = BMI2_DRDY_INT;

  /* List the sensors which are required to enable */
  uint8_t sens_list = BMI2_ACCEL;

  /* Structure to define the type of the sensor and its configurations */
  struct bmi2_sens_config config;

  /* Configure type of feature */
  config.type = BMI2_ACCEL;

  /* Enable the selected sensors */
  rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

  if (rslt == BMI2_OK)
  {
    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {

      config.cfg.acc.odr = BMI2_ACC_ODR_25HZ;

      /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
      config.cfg.acc.range = BMI2_ACC_RANGE_2G;
      config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
      config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

      /* Set the gyro configurations */
      rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

      if (rslt == BMI2_OK)
      {
        /* Map interrupt to pins */
        rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);
      }
    }
  }

  return rslt;
}

int8_t BMI2_BMM1_Class::bmi2_gyro_set_config(struct bmi2_dev *bmi2_dev)
{
   /* Variable to define result */
    int8_t rslt;

    /* Initialize interrupts for gyroscope */
    uint8_t sens_int = BMI2_DRDY_INT;

    /* List the sensors which are required to enable */
    uint8_t sens_list = BMI2_GYRO;

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_GYRO;

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

        if (rslt == BMI2_OK)
        {
            /* The user can change the following configuration parameter according to their requirement */
            /* Output data Rate. By default ODR is set as 200Hz for gyro */
            config.cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
            /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps */
            config.cfg.gyr.range = BMI2_GYR_RANGE_2000;
            config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            /* Set the gyro configurations */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

            if (rslt == BMI2_OK)
            {
                /* Map interrupt to pins */
                rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);
            }
        }
    }

    return rslt;
}

int8_t BMI2_BMM1_Class::bmi2_mag_set_config(struct bmm150_dev *dev)
{
     uint8_t sensor_list[] = {  BMI2_AUX };
    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;
    int8_t rslt;

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config config[1];

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    config[0].type = BMI2_AUX;

    /* Pull-up resistor 2k is set to the trim regiter */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi2);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 1, &bmi2);

    /* Configurations for aux. */
    config[0].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    config[0].cfg.aux.aux_en = BMI2_ENABLE;
    config[0].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[0].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[0].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    config[0].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_set_sensor_config(config, 1, &bmi2);

    /* NOTE:
    *   Accel and gyro enable must be done after setting configurations
    */
    rslt = bmi270_sensor_enable(sensor_list, 1, &bmi2);

    /* Initialize bmm150. */
    rslt = bmm150_init(dev);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);

    rslt = bmi270_get_sensor_config(config, 1, &bmi2);

    /* Disable manual mode so that the data mode is enabled. */
    config[0].cfg.aux.manual_en = BMI2_DISABLE;

    /* Set the aux configurations. */
    rslt = bmi270_set_sensor_config(config, 1, &bmi2);
 

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi2);


    struct bmi2_int_pin_config int_pin_conf;

    bmi2_get_int_pin_config(&int_pin_conf, &bmi2);

    int_pin_conf.pin_type = BMI2_INT1;
    //int_pin_conf.int_latch=BMI2_INT_LATCH;
    int_pin_conf.pin_cfg[0/*INT1*/] = {
        .lvl = BMI2_INT_ACTIVE_HIGH,
        .od = BMI2_INT_PUSH_PULL,
        .output_en = BMI2_INT_OUTPUT_ENABLE,
        .input_en = BMI2_INT_INPUT_DISABLE
    } ;

    rslt = bmi2_set_int_pin_config(&int_pin_conf, &bmi2);

    if (dev->chip_id == BMM150_CHIP_ID) {
        _debug->println("Valid BMM150 (Aux) sensor");
    }
    return rslt;
}
        
int BMI2_BMM1_Class::accelerationAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_ACC_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_ACC_DRDY_INT_MASK;
  return ret;
}

float BMI2_BMM1_Class::accelerationSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_ACCEL;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.acc.odr) * 0.39;
}

// default range is +-2000dps, so conversion factor is (((1 << 15)/4.0f))
#define INT16_to_DPS   (16.384f)

int BMI2_BMM1_Class::gyroscopeAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_GYR_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_GYR_DRDY_INT_MASK;
  return ret;
}

int BMI2_BMM1_Class::magneticFieldAvailable() {
  uint16_t status;
  bmi2_get_int_status(&status, &bmi2);
  int ret = ((status | _int_status) & BMI2_AUX_DRDY_INT_MASK);
  _int_status = status;
  _int_status &= ~BMI2_AUX_DRDY_INT_MASK;
  return ret;
  
}

float BMI2_BMM1_Class::gyroscopeSampleRate() {
  struct bmi2_sens_config sens_cfg;
  sens_cfg.type = BMI2_GYRO;
  bmi2_get_sensor_config(&sens_cfg, 1, &bmi2);
  return (1 << sens_cfg.cfg.gyr.odr) * 0.39;
}

int BMI2_BMM1_Class::readGyroAccel(imu_data_t & imu_sensor_data, bool raw){ // Results are in G (earth gravity). // Results are in degrees/second.
struct bmi2_sens_data sensor_data = { { 0 } };

    int res = bmi2_get_sensor_data(&sensor_data, &bmi2);

   if(raw == false){
   imu_sensor_data.acc.x = sensor_data.acc.x* 9.8 / BMI2_ACC_FOC_2G_REF;
    imu_sensor_data.acc.y  = sensor_data.acc.y* 9.8 / BMI2_ACC_FOC_2G_REF;
    imu_sensor_data.acc.z  = sensor_data.acc.z* 9.8 / BMI2_ACC_FOC_2G_REF;
    
     imu_sensor_data.gyr.x  = sensor_data.gyr.x / BMI2_GYRO_FOC_2000_DPS_REF;
    imu_sensor_data.gyr.y = sensor_data.gyr.y / BMI2_GYRO_FOC_2000_DPS_REF;
    imu_sensor_data.gyr.z = sensor_data.gyr.z / BMI2_GYRO_FOC_2000_DPS_REF;
}
else // raw true
{
    imu_sensor_data.acc.x = sensor_data.acc.x;
    imu_sensor_data.acc.y  = sensor_data.acc.y;
    imu_sensor_data.acc.z  = sensor_data.acc.z;
    
    imu_sensor_data.gyr.x  = sensor_data.gyr.x;
    imu_sensor_data.gyr.y = sensor_data.gyr.y;
    imu_sensor_data.gyr.z = sensor_data.gyr.z;
}
    return res;
    
}

int BMI2_BMM1_Class::readAcceleration(float& x, float& y, float& z){ // Results are in G (earth gravity). // Results are in degrees/second.
struct bmi2_sens_data sensor_data = { { 0 } };

    int res = bmi2_get_sensor_data(&sensor_data, &bmi2);

   x = sensor_data.acc.x* 9.8 / BMI2_ACC_FOC_2G_REF;

    y  = sensor_data.acc.y* 9.8 / BMI2_ACC_FOC_2G_REF;

    z  = sensor_data.acc.z* 9.8 / BMI2_ACC_FOC_2G_REF;

    return res;
    
}

int BMI2_BMM1_Class::readGyroscope(float& x, float& y, float& z){ 
struct bmi2_sens_data sensor_data = { { 0 } };

    int res = bmi2_get_sensor_data(&sensor_data, &bmi2);
    
     x  = sensor_data.gyr.x / BMI2_GYRO_FOC_2000_DPS_REF;

    y = sensor_data.gyr.y / BMI2_GYRO_FOC_2000_DPS_REF;

    z = sensor_data.gyr.z / BMI2_GYRO_FOC_2000_DPS_REF;

    return res;
    
}

int BMI2_BMM1_Class::readMagneticField(float& x, float& y, float& z){ // Results are in uT (micro Tesla).

mag_data_t mag_sensor_data;
int res = readAuxMag(mag_sensor_data);

x = mag_sensor_data.x ;
y = mag_sensor_data.y ;
z = mag_sensor_data.z ;

return res;
}


int BMI2_BMM1_Class::readAuxMag(mag_data_t & mag_sensor_data){ // Results are in uT (micro Tesla).

struct bmi2_sens_data sensor_data = { { 0 } };

int res = bmi2_get_sensor_data(&sensor_data, &bmi2);
    
struct bmm150_mag_data mag_data;
res= bmm150_aux_mag_data(sensor_data.aux_data, &mag_data, &bmm1);

mag_sensor_data.x = mag_data.x ;
mag_sensor_data.y = mag_data.y;
mag_sensor_data.z = mag_data.z;

return res;
}

int BMI2_BMM1_Class::BMI2_BMM1_Class::magneticFieldSampleRate() {
//TODO
  return 0;
}

void BMI2_BMM1_Class::print_rslt(int8_t rslt)
{
  switch (rslt)
  {
    case BMI2_OK: return; /* Do nothing */ break;
    case BMI2_E_NULL_PTR:
      _debug->println("Error [" + String(rslt) + "] : Null pointer");
       
      break;
    case BMI2_E_COM_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Communication failure");
       
      break;
    case BMI2_E_DEV_NOT_FOUND:
      _debug->println("Error [" + String(rslt) + "] : Device not found");
       
      break;
    case BMI2_E_OUT_OF_RANGE:
      _debug->println("Error [" + String(rslt) + "] : Out of range");
       
      break;
    case BMI2_E_ACC_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel configuration");
       
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid gyro configuration");
       
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
       
      break;
    case BMI2_E_INVALID_SENSOR:
      _debug->println("Error [" + String(rslt) + "] : Invalid sensor");
       
      break;
    case BMI2_E_CONFIG_LOAD:
      _debug->println("Error [" + String(rslt) + "] : Configuration loading error");
       
      break;
    case BMI2_E_INVALID_PAGE:
      _debug->println("Error [" + String(rslt) + "] : Invalid page ");
       
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      _debug->println("Error [" + String(rslt) + "] : Invalid feature bit");
       
      break;
    case BMI2_E_INVALID_INT_PIN:
      _debug->println("Error [" + String(rslt) + "] : Invalid interrupt pin");
       
      break;
    case BMI2_E_SET_APS_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
       
      break;
    case BMI2_E_AUX_INVALID_CFG:
      _debug->println("Error [" + String(rslt) + "] : Invalid auxiliary configuration");
       
      break;
    case BMI2_E_AUX_BUSY:
      _debug->println("Error [" + String(rslt) + "] : Auxiliary busy");
       
      break;
    case BMI2_E_SELF_TEST_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Self test failed");
       
      break;
    case BMI2_E_REMAP_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Remapping error");
       
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      _debug->println("Error [" + String(rslt) + "] : Gyro user gain update failed");
       
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      _debug->println("Error [" + String(rslt) + "] : Self test not done");
       
      break;
    case BMI2_E_INVALID_INPUT:
      _debug->println("Error [" + String(rslt) + "] : Invalid input");
       
      break;
    case BMI2_E_INVALID_STATUS:
      _debug->println("Error [" + String(rslt) + "] : Invalid status");
       
      break;
    case BMI2_E_CRT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : CRT error");
       
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      _debug->println("Error [" + String(rslt) + "] : Self test already running");
       
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      _debug->println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
       
      break;
    case BMI2_E_DL_ERROR:
      _debug->println("Error [" + String(rslt) + "] : DL error");
       
      break;
    case BMI2_E_PRECON_ERROR:
      _debug->println("Error [" + String(rslt) + "] : PRECON error");
       
      break;
    case BMI2_E_ABORT_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Abort error");
       
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test error");
       
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Gyro self test timeout");
       
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      _debug->println("Error [" + String(rslt) + "] : Write cycle ongoing");
       
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      _debug->println("Error [" + String(rslt) + "] : Write cycle timeout");
       
      break;
    case BMI2_E_ST_NOT_RUNING:
      _debug->println("Error [" + String(rslt) + "] : Self test not running");
       
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      _debug->println("Error [" + String(rslt) + "] : Data ready interrupt failed");
       
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      _debug->println("Error [" + String(rslt) + "] : Invalid FOC position");
       
      break;
    default:
      _debug->println("Error [" + String(rslt) + "] : Unknown error code");
       
      break;
  }
}

BMI2_BMM1_Class IMU_BMI270_BMM150(Wire);

