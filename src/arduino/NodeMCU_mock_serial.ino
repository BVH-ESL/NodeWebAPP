//////////////////////////////////////////////////////////////////////////////
// Author: RSP @ ESL (Embedded System Lab), KMUTNB, Thailand
// Date: 2017-March-20
// Board: LinkIt Smart 7688 Duo
// Arduino IDE: version 1.6.9 + LinkIt Smart 7688 Duo BoardManager 0.1.8
// http://download.labs.mediatek.com/package_mtk_linkit_smart_7688_test_index.json
//////////////////////////////////////////////////////////////////////////////
// note: The Arduino driver for the Serial port uses a 64 byte buffers for transmit and receive.

// Possible USB CDC baudrates:
//   115200, 230400, 460800, 500000, 576000, 21600, 1000000, 1152000,
//   1500000, 2000000, 2500000, 3000000, 3500000, 4000000

#define NUM_SAMPLES      (1000)
#define INTERVAL_USEC    (5000UL)
#define LINE_BUF_SIZE    (32)
#define extInterrupt     (5)      //D1

char sbuf[64];
uint16_t samples = 0;

char line_buf[ LINE_BUF_SIZE+1 ];
uint8_t line_buf_index = 0;
uint32_t ts, start_ts;

enum states {
  ST_NONE=0, ST_RST, ST_ARMED, ST_SAMPLING, ST_READY
};

enum commands {
  CMD_NONE=0, CMD_ARM, CMD_DISARM
};

typedef enum states state_t;
state_t state = ST_RST;

boolean ext_irq_enabled = false;
volatile boolean ext_irq_detected = false;

void trigger_isr( ) {
  if (!ext_irq_detected) {
    ext_irq_detected = true;
  }
}

void sendData() {
  // read two analog channels A0 and A1
  // uint16_t a0 = analogRead(A0);
  // uint16_t a1 = analogRead(A1);
  sprintf( sbuf, "%04d,%u,%u,%lu\n", samples, 102, 1150, micros() );
  Serial.print( sbuf );
  Serial.flush();
}

int getNextCmd() {
  int cmd = CMD_NONE;
  if ( Serial.available() > 0 ) {
     char ch = Serial.read(); // read one byte from the MT7688 MPU
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
  Serial.begin(921600);
  // Serial1.begin(500000); // MPU-MCU via serial
  delay(1000);
}

void loop() {
  int cmd = getNextCmd();

  switch (state) {
    case ST_RST:
       if ( cmd == CMD_ARM ) {
          state = ST_ARMED;  // go to state ST_ARMED
          samples = 0;
       }
       break;

    case ST_ARMED:
       if ( cmd == CMD_DISARM ) {
          state = ST_RST;
          detachInterrupt( extInterrupt ); // use EINT0 / D3 input pin on 32u4
          ext_irq_enabled  = false;
          ext_irq_detected = false;
          delay(5);
          // Serial.println("disarmed");
          Serial.println("#DISARMED");
       }
       else if ( !ext_irq_enabled ) {
          state = ST_ARMED;
          attachInterrupt( extInterrupt, trigger_isr, FALLING ); // use EINT0 / D3 input pin on 32u4
          ext_irq_enabled  = true;
          ext_irq_detected = false;
          delay(5);
          // Serial.println("armed");
          Serial.println("#ARMED");
       }

       if ( ext_irq_detected ) {
          state = ST_SAMPLING; // go to state ST_SAMPLING
          detachInterrupt( extInterrupt ); // use EINT0 / D3 input pin on 32u4
          ext_irq_enabled  = false;
          ext_irq_detected = false;
          samples = 0;
          // Serial.println("sampling started");
          Serial.println("#START");
          ts = micros();
          start_ts = ts;
       }
       break;

    case ST_SAMPLING:
       if ( micros() - ts >= INTERVAL_USEC ) { // run every 4 msec.
         sendData();
         ts += INTERVAL_USEC;
         samples++;
         if ( samples >= NUM_SAMPLES ) {
            state = ST_READY; // go to state ST_READY
            delay(5);
            // Serial.println("sampling ended");
            Serial.println("#END");
         }
       }
       break;

    case ST_READY:
       state = ST_RST;
       delay(5);
      //  Serial.println("ready");
       Serial.println("#READY");
       break;

    default:
       state = ST_RST;
       break;
  }
}
///////////////////////////////////////////////////////////////////////////////
