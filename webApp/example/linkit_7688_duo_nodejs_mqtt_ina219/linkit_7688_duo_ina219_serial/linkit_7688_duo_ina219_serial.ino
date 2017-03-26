//////////////////////////////////////////////////////////////////////////////
// Author: RSP @ ESL (Embedded System Lab), KMUTNB, Thailand
// Date: 2017-March-24
// Board: LinkIt Smart 7688 Duo
// Arduino IDE: version 1.6.9 + LinkIt Smart 7688 Duo BoardManager 0.1.8
// http://download.labs.mediatek.com/package_mtk_linkit_smart_7688_test_index.json
//////////////////////////////////////////////////////////////////////////////
// note: The Arduino driver for the Serial port uses a 64 byte buffers for transmit and receive. 

// Possible USB CDC baudrates: 
//   115200, 230400, 460800, 500000, 576000, 21600, 1000000, 1152000, 
//   1500000, 2000000, 2500000, 3000000, 3500000, 4000000

#include <Wire.h>
#include "Adafruit_INA219.h"

Adafruit_INA219 ina219;

#define NUM_SAMPLES      (4000)
#define INTERVAL_USEC    (5000UL)
#define LINE_BUF_SIZE    (32)

char sbuf[80];
uint16_t samples = 0;

char line_buf[ LINE_BUF_SIZE+1 ];
uint8_t line_buf_index = 0;
uint32_t ts, start_ts, dT = 0;

const int EINT_PIN = 7; // D7 <=> EINT4
const int EINT_NUM = 4;

enum states   { ST_NONE=0, ST_RST, ST_ARMED, ST_SAMPLING, ST_READY };
enum commands { CMD_NONE=0, CMD_ARM, CMD_DISARM };

typedef enum states state_t;
state_t state = ST_RST;

boolean ext_irq_enabled = false;
volatile boolean ext_irq_detected = false;
volatile uint8_t trigger_cnt = 0;

void trigger_isr( ) {
  if ( !ext_irq_detected && (digitalRead(EINT_PIN) == LOW) ) {
    ext_irq_detected = true; 
    trigger_cnt++;
  }
}

int32_t sensor_1mV, sensor_100uA;

void sendData() {
  sensor_1mV   = ina219.getBusVoltage_mV();
  sensor_100uA = ina219.getCurrent_100uA() - 8;
  sprintf( sbuf, "%4d,%d,%d.%d,%u,%u\n", samples,
          (int16_t)(sensor_1mV), (int16_t)(sensor_100uA/10), (int16_t)(sensor_100uA%10), 
          trigger_cnt, (uint16_t)dT );
  Serial1.print( sbuf );
  Serial1.flush();
}

int getNextCmd() {
  int cmd = CMD_NONE;
  if ( Serial1.available() > 0 ) {
     char ch = Serial1.read(); // read one byte from the MT7688 MPU
     if ( (line_buf_index < LINE_BUF_SIZE) && (ch != '\n') ) {
        line_buf[ line_buf_index++ ] = ch;
     }
     if ( ch == '\n' ) {
        line_buf[ line_buf_index ] = '\0';
        line_buf_index = 0;
        if ( !strcmp(line_buf, "ARM") ) {
           cmd = CMD_ARM;
        } 
        else if ( !strcmp(line_buf, "DISARM") ) {
           cmd = CMD_DISARM;
        } 
     }
  }
  return cmd;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(500000); // MPU-MCU via serial
  Wire.begin();
  ina219.begin();
  Wire.setClock( 400000 );
  delay(100);
  ina219.setCalibration_16V_800mA();
  delay(1000);
}

void loop() {
  int cmd = getNextCmd();
  
  switch (state) {
    case ST_RST:
       if ( cmd == CMD_ARM ) {
          state = ST_ARMED;  // go to state ST_ARMED
          samples = 0;
          trigger_cnt = 0;
       }
       break;

    case ST_ARMED:
       if ( cmd == CMD_DISARM ) {
          state = ST_RST;
          detachInterrupt( EINT_NUM ); 
          ext_irq_enabled = false;
          delay(5);
          Serial.println("disarmed");
          Serial1.println("#DISARMED");
       }
       else if ( !ext_irq_enabled ) {
          state = ST_ARMED;
          attachInterrupt( EINT_NUM, trigger_isr, FALLING ); 
          ext_irq_enabled = true;
          delay(5);
          Serial.println("armed");
          Serial1.println("#ARMED");
       } 
       
       if ( ext_irq_detected ) {
          state = ST_SAMPLING; // go to state ST_SAMPLING
          samples = 0;
          Serial.println("sampling started");
          Serial1.println("#START");
          ts = micros();
       }
       break;
       
    case ST_SAMPLING:
       if ( micros() - ts >= INTERVAL_USEC ) {
         start_ts = micros();
         sendData();
         ts += INTERVAL_USEC;
         samples++;
         dT = micros() - start_ts;         
         if ( samples >= NUM_SAMPLES ) {
            state = ST_READY; // go to state ST_READY
            delay(5);
            Serial.println("sampling ended");
            Serial1.println("#END");
         }
       }
       ext_irq_detected = false;
       break;
       
    case ST_READY:
       state = ST_RST;
       detachInterrupt( EINT_NUM );
       ext_irq_enabled = false;
       ext_irq_detected = false;
       delay(5);
       Serial.println("ready");
       Serial1.println("#READY");
       break;

    default:
       state = ST_RST;
       break;
  }
}
///////////////////////////////////////////////////////////////////////////////

