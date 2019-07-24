#ifndef _TMP117_H_
#define _TMP117_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "SDK_EVAL_I2C.h"

 // Device address
#define TMP117_Address 0x48   

#define TMP117_REG_TEMPERATURE          0x00
#define TMP117_REG_CONFIGURATION        0x01
#define TMP117_REG_TEMP_HIGH_LIMIT      0x02
#define TMP117_REG_TEMP_LOW_LIMIT       0x03

#define TMP117_REG_EEPROM_UNLOCK        0x04
#define TMP117_REG_EEPROM1              0x05
#define TMP117_REG_EEPROM2              0x06
#define TMP117_REG_EEPROM3              0x08

#define TMP117_REG_TEMPERATURE_OFFSET   0x07
#define TMP117_REG_DEVICE_ID            0x0F

#define TMP117_RESOLUTION               (double)0.0078125

//typedef void (*allert_callback)(void);

/*  Conversion Cycle Time in CC Mode
              AVG       0       1       2       3
      CONV  averaging  (0)     (8)     (32)   (64)
        0             15.5ms  125ms   500ms    1s     C15mS5
        1             125ms   125ms   500ms    1s     C125mS
        2             250ms   250ms   500ms    1s     C250mS
        3             500ms   500ms   500ms    1s     C500mS
        4             1s      1s      1s       1s     C1S
        5             4s      4s      4s       4s     C4S
        6             8s      8s      8s       8s     C8S
        7             16s     16s     16s      16s    C16S
*/

typedef enum {THERMAL = 0, ALERT, DATA}TMP117_PMODE;                                 //!<  Pin mode 
typedef enum {CONTINUOUS = 0, SHUTDOWN = 1, ONESHOT = 3}TMP117_CMODE;                //!<  Conversion mode 
typedef enum {C15mS5 = 0, C125mS, C250mS, C500mS, C1S, C4S, C8S, C16S}TMP117_CONVT;  //!<  Conversion time
typedef enum {NOAVE = 0, AVE8, AVE32, AVE64}TMP117_AVE;                               //!<  Averaging mode
typedef enum {NOALERT = 0, HIGHALERT, LOWALERT}TMP117_ALERT;                         //!<  Alert type 
typedef enum {ACTIVE_H = 0, ACTIVE_L}TMP117_POL;                                     //!<  Polarity Alert

void      TMP117_update (void);
void      TMP117_softReset ( void );

void      TMP117_setAlertMode ( TMP117_PMODE mode);
void      TMP117_setAllertCallback ( void (*allert_callback)(void), uint8_t pin );
void      TMP117_setAllertTemperature ( double lowtemp, double hightemp );
void      TMP117_setConvMode ( TMP117_CMODE cmode);
void      TMP117_setConvTime ( TMP117_CONVT convtime );
void      TMP117_setAveraging ( TMP117_AVE ave );
void      TMP117_setOffsetTemperature ( double offset );
void      TMP117_setTargetTemperature ( double target );
void TMP117_SetConfig(TMP117_CONVT convtime, TMP117_AVE ave, 
                      TMP117_PMODE mode, TMP117_CMODE cmode, TMP117_POL pol);

double    TMP117_getTemperature ( void );
uint16_t  TMP117_getDeviceID ( void );
uint16_t  TMP117_getDeviceRev ( void );
double    TMP117_getOffsetTemperature ( void );
TMP117_ALERT TMP117_getAlertType ( void );
void      TMP117_writeEEPROM ( uint16_t data, uint8_t eeprom_nr );
uint16_t  TMP117_readEEPROM ( uint8_t eeprom_nr );
uint16_t  TMP117_readConfig ( void );
void      TMP117_i2cWrite2B ( uint8_t reg, uint16_t data );
uint16_t  TMP117_i2cRead2B ( uint8_t reg );
void      TMP117_writeConfig ( uint16_t config_data );
void      TMP117_printConfig ( uint16_t reg_value );
void      TMP117_lockEEPROM ( void );
void      TMP117_unlockEEPROM ( void );
uint8_t   TMP117_EEPROMisBusy ( void );


    
#endif