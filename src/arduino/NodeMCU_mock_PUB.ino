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

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// init WiFi AP
const char *ssid =	"ESL_Lab1";		// cannot be longer than 32 characters!
const char *pass =	"wifi@esl";		//

IPAddress server(192, 168, 1, 182);

WiFiClient wclient;
PubSubClient client(wclient, server);

#define NUM_SAMPLES      (1000)
#define INTERVAL_USEC    (2500UL)
#define LINE_BUF_SIZE    (32)
#define extInterrupt     (4)      //D2
#define NODE_ID          (1)

// define GPIO variable
#define scl              (14)                      // D0 pin
#define sda              (12)                       // D1 pin
#define ledR             (0)                       // D3
#define ledG             (2)                       // D4

char sbuf[64];
uint16_t samples = 0;

char line_buf[ LINE_BUF_SIZE+1 ];
uint8_t line_buf_index = 0;
uint8_t cmd_size = 0;
int cmd = 0;
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
static unsigned long stateTime = micros();

void checkCmd(String input){
  // Serial.println(input);
  cmd = CMD_NONE;
  if (input == "ARM"){
    cmd = CMD_ARM;
  }else if (input == "DISARM"){
    cmd = CMD_DISARM;
  }
}

void callback(const MQTT::Publish& pub) {
  if (pub.has_stream()) {
    int read;
    while (read = pub.payload_stream()->read((uint8_t *)line_buf, LINE_BUF_SIZE)) {
      Serial.write(line_buf, read);
    }
    pub.payload_stream()->stop();
    Serial.println("");
  } else{
    checkCmd(pub.payload_string());
  }
}

void trigger_isr( ) {
  if (!ext_irq_detected) {
    ext_irq_detected = true;
  }
}

void sendData() {
  sprintf( sbuf, "%04d,%u,%u,%lu\n", samples, 102, 1150, micros() - stateTime );
  stateTime = micros();
  client.publish(NODE_ID+"/data",sbuf);
}

void setup() {
  Serial.begin(921600);

  pinMode(extInterrupt, INPUT_PULLUP);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, LOW);
  Wire.begin(sda, scl); // use I2C port
  Wire.setClock(400000); // init scl speed 400kHz

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  // if (WiFi.status() == WL_CONNECTED) {
  //   if (!client.connected()) {
  //     if (client.connect("arduinoClient")) {
	//       client.set_callback(callback);
	//       client.subscribe("cmd");
  //     }
  //   }
  //
  //   if (client.connected())
  //     client.loop();
  // }

  // Serial1.begin(500000); // MPU-MCU via serial
  delay(1000);
}

void loop() {
  // check MQTT connection to broker
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      if (client.connect("arduinoClient")) {
	      client.set_callback(callback);
	      client.subscribe("cmd");
      }
    }

    if (client.connected())
      client.loop();
  }

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
          // Serial.println("#DISARMED");
          client.publish(NODE_ID+"/status","#DISARMED");
          delay(5);
          // Serial.println("armed");
          // Serial.println("#ARMED");
          client.publish(NODE_ID+"/status","#ARMED");
          digitalWrite(ledG, HIGH);
       }

       if ( ext_irq_detected ) {
          state = ST_SAMPLING; // go to state ST_SAMPLING
          detachInterrupt( extInterrupt ); // use EINT0 / D3 input pin on 32u4
          ext_irq_enabled  = false;
          ext_irq_detected = false;
          samples = 0;
          // Serial.println("sampling started");
          // Serial.println("#START");
          client.publish(NODE_ID+"/status","#START");
          digitalWrite(ledG, LOW);
          ts = micros();
          start_ts = ts;
       }
       break;

    case ST_SAMPLING:
       if ( micros() - ts >= INTERVAL_USEC ) { // run every 4 msec.
         sendData();
         ts += INTERVAL_USEC;
         samples++;
         if(samples % 50 == 0){
           digitalWrite(ledR, !digitalRead(ledR));
         }
         if ( samples >= NUM_SAMPLES ) {
            state = ST_READY; // go to state ST_READY
            delay(5);
            // Serial.println("sampling ended");
            // Serial.println("#END");
            client.publish(NODE_ID+"/status","#END");
            digitalWrite(ledR, LOW);
         }
       }
       break;

    case ST_READY:
       state = ST_RST;
       delay(5);
      //  Serial.println("ready");
      //  Serial.println("#READY");
       client.publish(NODE_ID+"/status","#READY");
       digitalWrite(ledG, HIGH);
       break;

    default:
       state = ST_RST;
       break;
  }
}
///////////////////////////////////////////////////////////////////////////////
