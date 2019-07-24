#include <stdlib.h>
#include "TSL2591.h"

  tsl2591IntegrationTime_t _integration;
  tsl2591Gain_t _gain;
  uint8_t _initialized = 0x00;
  
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit TSL2591 class
    @param  sensorID An optional ID # so you can track this sensor, it will tag sensorEvents you create.
*/
/**************************************************************************/
//TSL2591_Adafruit_TSL2591(int32_t sensorID)
//{
//  _initialized = 0x00;
//  _integration = TSL2591_INTEGRATIONTIME_100MS;
//  _gain        = TSL2591_GAIN_MED;
//  _sensorID    = sensorID;
//
//  // we cant do wire initialization till later, because we havent loaded Wire yet
//}

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware, identifies if chip is found
    @returns 0x01 if a TSL2591 is found, 0x00 on any failure
*/
/**************************************************************************/
uint8_t TSL2591_begin(void)
{
  
  uint8_t id = TSL2591_read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_ID);
  if (id != 0x50 ) {
    return 0x00;
  }
  // Serial.println("Found Adafruit_TSL2591");

  _initialized = 0x01;

  // Set default integration time and gain
  TSL2591_setTiming(TSL2591_INTEGRATIONTIME_100MS);
  TSL2591_setGain(TSL2591_GAIN_MED);

  // Note: by default, the device is in power down mode on bootup
  TSL2591_disable();

  return 0x01;
}

/**************************************************************************/
/*!
    @brief  Enables the chip, so it's ready to take readings
*/
/**************************************************************************/
void TSL2591_enable(void)
{
  if (!_initialized)
  {
    if (!TSL2591_begin())
    {
      return;
    }
  }

  // Enable the device by setting the control bit to 0x01
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE,
	 TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN | TSL2591_ENABLE_AIEN | TSL2591_ENABLE_NPIEN);
}


/**************************************************************************/
/*!
    @brief Disables the chip, so it's in power down mode
*/
/**************************************************************************/
void TSL2591_disable(void)
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return;
    }
  }

  // Disable the device by setting the control bit to 0x00
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_ENABLE, TSL2591_ENABLE_POWEROFF);
}

/************************************************************************/
/*!
    @brief  Setter for sensor light gain
    @param  gain {@link tsl2591Gain_t} gain value
*/
/**************************************************************************/
void TSL2591_setGain(tsl2591Gain_t gain)
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return;
    }
  }

  TSL2591_enable();
  _gain = gain;
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, (uint8_t)_integration |(uint8_t)_gain);
  TSL2591_disable();
}

/************************************************************************/
/*!
    @brief  Getter for sensor light gain
    @returns {@link tsl2591Gain_t} gain value
*/
/**************************************************************************/
tsl2591Gain_t TSL2591_getGain()
{
  return _gain;
}

/************************************************************************/
/*!
    @brief  Setter for sensor integration time setting
    @param integration {@link tsl2591IntegrationTime_t} integration time setting
*/
/**************************************************************************/
void TSL2591_setTiming(tsl2591IntegrationTime_t integration)
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return;
    }
  }

  TSL2591_enable();
  _integration = integration;
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CONTROL, (uint8_t)_integration | (uint8_t)_gain);
  TSL2591_disable();
}

/************************************************************************/
/*!
    @brief  Getter for sensor integration time setting
    @returns {@link tsl2591IntegrationTime_t} integration time
*/
/**************************************************************************/
tsl2591IntegrationTime_t TSL2591_getTiming()
{
  return _integration;
}

/************************************************************************/
/*!
    @brief  Calculates the visible Lux based on the two light sensors
    @param  ch0 Data from channel 0 (IR+Visible)
    @param  ch1 Data from channel 1 (IR)
    @returns Lux, based on AMS coefficients (or < 0 if overflow)
*/
/**************************************************************************/
float TSL2591_calculateLux(uint16_t ch0, uint16_t ch1)
{
  float    atime, again;
  float    cpl, lux1, lux2, lux;
  uint32_t chan0, chan1;

  // Check for overflow conditions first
  if ((ch0 == 0xFFFF) | (ch1 == 0xFFFF))
  {
    // Signal an overflow
    return -1;
  }

  // Note: This algorithm is based on preliminary coefficients
  // provided by AMS and may need to be updated in the future

  switch (_integration)
  {
    case TSL2591_INTEGRATIONTIME_100MS :
      atime = 100.0F;
      break;
    case TSL2591_INTEGRATIONTIME_200MS :
      atime = 200.0F;
      break;
    case TSL2591_INTEGRATIONTIME_300MS :
      atime = 300.0F;
      break;
    case TSL2591_INTEGRATIONTIME_400MS :
      atime = 400.0F;
      break;
    case TSL2591_INTEGRATIONTIME_500MS :
      atime = 500.0F;
      break;
    case TSL2591_INTEGRATIONTIME_600MS :
      atime = 600.0F;
      break;
    default: // 100ms
      atime = 100.0F;
      break;
  }

  switch (_gain)
  {
    case TSL2591_GAIN_LOW :
      again = 1.0F;
      break;
    case TSL2591_GAIN_MED :
      again = 25.0F;
      break;
    case TSL2591_GAIN_HIGH :
      again = 428.0F;
      break;
    case TSL2591_GAIN_MAX :
      again = 9876.0F;
      break;
    default:
      again = 1.0F;
      break;
  }

  // cpl = (ATIME * AGAIN) / DF
  cpl = (atime * again) / TSL2591_LUX_DF;

  // Original lux calculation (for reference sake)
  //lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
  //lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
  //lux = lux1 > lux2 ? lux1 : lux2;

  // Alternate lux calculation 1
  // See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
  lux = ( ((float)ch0 - (float)ch1 )) * (1.0F - ((float)ch1/(float)ch0) ) / cpl;

  // Alternate lux calculation 2
  //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;

  // Signal I2C had no errors
  return lux;
}

/************************************************************************/
/*!
    @brief  Reads the raw data from both light channels
    @returns 32-bit raw count where high word is IR, low word is IR+Visible
*/
/**************************************************************************/
uint32_t TSL2591_getFullLuminosity (void)
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return 0;
    }
  }

  // Enable the device
  TSL2591_enable();

  // Wait x ms for ADC to complete
  for (uint8_t d=0; d<=_integration; d++)
  {
    Clock_Wait(120);
  }

  // CHAN0 must be read before CHAN1
  // See: https://forums.adafruit.com/viewtopic.php?f=19&t=124176
  uint32_t x;
  uint16_t y=0;
  y |= TSL2591_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN0_LOW);
  x = TSL2591_read16(TSL2591_COMMAND_BIT | TSL2591_REGISTER_CHAN1_LOW);
  x <<= 16;
  x |= y;

  TSL2591_disable();

  return x;
}

/************************************************************************/
/*!
    @brief  Reads the raw data from the channel
    @param  channel Can be 0 (IR+Visible, 1 (IR) or 2 (Visible only)
    @returns 16-bit raw count, or 0 if channel is invalid
*/
/**************************************************************************/
uint16_t TSL2591_getLuminosity (uint8_t channel)
{
  uint32_t x = TSL2591_getFullLuminosity();

  if (channel == TSL2591_FULLSPECTRUM)
  {
    // Reads two byte value from channel 0 (visible + infrared)
    return (x & 0xFFFF);
  }
  else if (channel == TSL2591_INFRARED)
  {
    // Reads two byte value from channel 1 (infrared)
    return (x >> 16);
  }
  else if (channel == TSL2591_VISIBLE)
  {
    // Reads all and subtracts out just the visible!
    return ( (x & 0xFFFF) - (x >> 16));
  }

  // unknown channel!
  return 0;
}

/************************************************************************/
/*!
    @brief  Set up the interrupt to go off when light level is outside the lower/upper range.
    @param  lowerThreshold Raw light data reading level that is the lower value threshold for interrupt
    @param  upperThreshold Raw light data reading level that is the higher value threshold for interrupt
    @param  persist How many counts we must be outside range for interrupt to fire, default is any single value
*/
/**************************************************************************/
void TSL2591_registerInterrupt(uint16_t lowerThreshold, uint16_t upperThreshold, tsl2591Persist_t persist)
{
  persist = TSL2591_PERSIST_ANY;
  
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return;
    }
  }

  TSL2591_enable();
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_PERSIST_FILTER,  persist);
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTL, lowerThreshold);
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AILTH, lowerThreshold >> 8);
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTL, upperThreshold);
  TSL2591_write8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_THRESHOLD_AIHTH, upperThreshold >> 8);
  TSL2591_disable();
}

/************************************************************************/
/*!
    @brief  Clear interrupt status
*/
/**************************************************************************/
void TSL2591_clearInterrupt()
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return;
    }
  }

  TSL2591_enable();
  //TSL2591_write8(TSL2591_CLEAR_INT);
  TSL2591_disable();
}


/************************************************************************/
/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte. Bit 0 is ALS Valid. Bit 4 is ALS Interrupt. Bit 5 is No-persist Interrupt.
*/
/**************************************************************************/
uint8_t TSL2591_getStatus(void)
{
  if (!_initialized) {
    if (!TSL2591_begin()) {
      return 0;
    }
  }

  // Enable the device
  TSL2591_enable();
  uint8_t x;
  x = TSL2591_read8(TSL2591_COMMAND_BIT | TSL2591_REGISTER_DEVICE_STATUS);
  TSL2591_disable();
  return x;
}

/************************************************************************/
/*!
    @brief  Gets the most recent sensor event
    @param  event Pointer to Adafruit_Sensor sensors_event_t object that will be filled with sensor data
    @return 0x01 on success, 0x00 on failure
*/
/**************************************************************************/
uint8_t TSL2591_getEvent(sensors_event_t *event)
{
  uint16_t ir, full;
  uint32_t lum ;
  /* Early silicon seems to have issues when there is a sudden jump in */
  /* light levels. :( To work around this for now sample the sensor 2x */
  lum = TSL2591_getFullLuminosity();
  ir = lum >> 16;
  full = lum & 0xFFFF;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 2591;
  event->type      = SENSOR_TYPE_LIGHT;
  event->timestamp = Clock_Time() ;   //millis();

  /* Calculate the actual lux value */
  /* 0 = sensor overflow (too much light) */
  event->light = TSL2591_calculateLux(full, ir);

  return 0x01;
}

/**************************************************************************/
/*!
    @brief  Gets the overall sensor_t data including the type, range and resulution
    @param  sensor Pointer to Adafruit_Sensor sensor_t object that will be filled with sensor type data
*/
/**************************************************************************/
void TSL2591_getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "TSL2591", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = 2591;
  sensor->type        = SENSOR_TYPE_LIGHT;
  sensor->min_delay   = 0;
  sensor->max_value   = 88000.0;
  sensor->min_value   = 0.0;
  sensor->resolution  = 0.001;
}
/*******************************************************/


uint8_t TSL2591_read8(uint8_t reg)
{
  uint8_t buffer;
  SdkEvalI2CRead(&buffer, TSL2591_ADDR, reg, 1);
  return buffer;
}

uint16_t TSL2591_read16(uint8_t reg)
{
  uint8_t buffer[2];
  uint16_t datac;
  SdkEvalI2CRead(buffer, TSL2591_ADDR, reg, 2);
  datac = ((buffer[0] << 8) | buffer[1]); 
  return datac;
}

void TSL2591_write8 (uint8_t reg, uint8_t value)
{
  SdkEvalI2CWrite(&value, TSL2591_ADDR, reg, 1);  
}


//void TSL2591_write8 (uint8_t reg)
//{
//  SdkEvalI2CWrite(&value, TSL2591_ADDR, reg, 1);  
//}