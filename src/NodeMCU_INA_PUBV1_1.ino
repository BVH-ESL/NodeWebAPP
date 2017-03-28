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

#define CALIB_VALUE         (20420)          // the calibration value
#define CURRENT_LSB         (20)             // uA per bit
#define SHUNT_VOLTAGE_LSB   (10)             // uV per bit
#define BUS_VOLTAGE_LSB     (4)              // mV per bit
#define POWER_LSB           (20*CURRENT_LSB) // uW per bit
#define CONFIG_VALUE        ( INA219_CONFIG_BV_MAX_16V \
                            | INA219_CONFIG_GAIN_2_80MV \
                            | INA219_CONFIG_BV_ADC_12BIT \
                            | INA219_CONFIG_SV_ADC_12BIT_1S_532US \
                            | INA219_CONFIG_MODE_SV_BV_CONTINUOUS )

const byte INA219_I2C_ADDR = 0b1000000; // 0x40 (7-bit I2C slave address)
// Default: A0=A1=GND (with on-board pull-down resistors)
// if the solder jumper is bridged -> logic '1'.
// A1=0,A0=0 => 0x40, A1=0,A0=1 => 0x41, A1=1,A0=0 => 0x44, and A1=1,A0=1 => 0x45

#define INA219_REG_CONFIG                    (0x00)
#define INA219_REG_SHUNT_VOLTAGE             (0x01)
#define INA219_REG_BUS_VOLTAGE               (0x02)
#define INA219_REG_POWER                     (0x03)
#define INA219_REG_CURRENT                   (0x04)
#define INA219_REG_CALIBRATION               (0x05)

#define INA219_CONFIG_GAIN_1_40MV            (0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV            (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV           (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV           (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_RST                    (0x8000)  // Soft Reset Bit
#define INA219_CONFIG_BV_MAX_16V             (0x0000)  // 16V Bus Voltage Full Range
#define INA219_CONFIG_BV_MAX_32V             (0x2000)  // 32V Bus Voltage Full Range
#define INA219_CONFIG_BV_ADC_12BIT           (0x0400)  // 12-bit Bus Voltage Sampling
#define INA219_CONFIG_SV_ADC_12BIT_1S_532US  (0x0018)  // 1x 12-bit Shunt Voltage Sampling
#define INA219_CONFIG_MODE_SV_BV_CONTINUOUS  (0x0007)  // Continuous Shunt & Bus Voltage Sampling

void INA219_readReg( uint8_t i2c_addr, uint8_t reg, uint16_t *value ) {
  uint8_t buf[2];
  uint8_t count = 0;

  Wire.beginTransmission( i2c_addr );
  Wire.write( reg );  // write the register address
  Wire.endTransmission();
  delayMicroseconds( 500 );
  Wire.requestFrom( i2c_addr, (uint8_t) 2 ); // read two bytes
  while ( Wire.available() && (count < 2) ) {
    buf[ count++ ] = Wire.read();
  }
  Wire.endTransmission();
  *value = ((uint16_t)buf[0]) << 8 | buf[1];
}

void INA219_writeReg( uint8_t i2c_addr, uint8_t reg, uint16_t value ) {
  Wire.beginTransmission( i2c_addr );
  Wire.write( reg );          // write the register address
  Wire.write( value >> 8 );   // write the higher byte
  Wire.write( value & 0xff ); // write the lower byte
  Wire.endTransmission();
}

void IN219_config() {
  uint16_t value;
  // Perform the soft reset
  value = INA219_CONFIG_RST;
  INA219_writeReg( INA219_I2C_ADDR, INA219_REG_CONFIG, value );
  delay(1);

  // Set the calibration register
  value = CALIB_VALUE;
  INA219_writeReg( INA219_I2C_ADDR, INA219_REG_CALIBRATION, value );

  // Set the config register
  value = CONFIG_VALUE;
  INA219_writeReg( INA219_I2C_ADDR, INA219_REG_CONFIG, value );
}

int32_t INA219_getShuntVoltage() { // in uV
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_SHUNT_VOLTAGE, &value );
  return ((int16_t)value) * (int32_t)SHUNT_VOLTAGE_LSB;
}

int16_t INA219_getBusVoltage() { // in mV
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_BUS_VOLTAGE, &value );
  return (((int16_t)value >> 3) * BUS_VOLTAGE_LSB /*mV*/ );
}

// getBus - getShunt

int32_t INA219_getCurrent() { // in  uA step
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_CURRENT, &value );
  return ((int16_t)value) * (int32_t)CURRENT_LSB;
}

int32_t INA219_getPower() { // Load Power = Current * Bus Voltage
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_POWER, &value);
  return (((int16_t)value) * (int32_t)POWER_LSB) / 1000;
}

// init WiFi AP
const char *ssid =	"ESL_Lab1";		// cannot be longer than 32 characters!
const char *pass =	"wifi@esl";		//
IPAddress server(192, 168, 1, 9);

// const char *ssid =	"test";		// cannot be longer than 32 characters!
// const char *pass =	"1qaz@WSX";		//
// IPAddress server(192, 168, 1, 101);

WiFiClient wclient;
PubSubClient client(wclient, server);

// #define NUM_SAMPLES      (1000)
// #define INTERVAL_USEC    (10000UL)
#define LINE_BUF_SIZE    (32)
#define extInterrupt     (0)      //D2
#define MIN_INTERVAL_USEC   (5000UL)
#define MAX_INTERVAL_USEC   (50000UL)
#define NUM_STATES (5)
// #define NODE_ID          (2)

int NUM_SAMPLES = 1000;
unsigned long INTERVAL_USEC = 5000;
int nodeID = 1;
char iTopic[32];
char oTopic[32];

// define GPIO variable
#define scl              (5)                      // D1 pin
#define sda              (4)                       // D2 pin
#define ledR             (13)                       // D7
#define ledG             (15)                       // D8
#define getProcessPin   (12)           // D6 pin
#define rstPin           (14)                   // D5 pin

uint8_t states = 0;

char sbuf[128];
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
  CMD_NONE=0, CMD_ARM, CMD_DISARM, CMD_SET
};

typedef enum states state_t;
state_t state = ST_RST;

boolean ext_irq_enabled = false;
volatile boolean ext_irq_detected = false;
static unsigned long startTime = micros();
static unsigned long endTime = micros();

int32_t currentSum = 0;
int32_t current = 0;
int32_t voltageSum = 0;
int32_t voltage = 0;
int32_t powerSum = 0;

// function check command
void checkCmd(String input){
  // Serial.println(input);
  cmd = CMD_NONE;
  if (input == "ARM"){
    cmd = CMD_ARM;
    // Serial.println(cmd);
  }else if (input == "DISARM"){
    cmd = CMD_DISARM;
  }else if(input == "CHECK"){
    sprintf(oTopic, "esl/%d/out/status", nodeID);
    client.publish(oTopic,"#ACK");
  }else{
    // Serial.println(input.substring(0, 3));
    if(input.substring(0,3) == "SET"){
      String temp = "";
      for(int i = 4; i<input.length(); i++){
        if(input[i] != ','){
          temp += input[i];
        }else{
          if(temp == "time"){
            // check min and max input interval
            if(input.substring(++i).toInt() < MIN_INTERVAL_USEC){
              INTERVAL_USEC = MIN_INTERVAL_USEC;
            }else if(input.substring(++i).toInt() > MAX_INTERVAL_USEC){
              INTERVAL_USEC = MAX_INTERVAL_USEC;
            }else{
              INTERVAL_USEC = input.substring(++i).toInt();
            }
            // Serial.println(INTERVAL_USEC);
            sprintf(oTopic, "esl/%d/out/status", nodeID);
            client.publish(oTopic,"#OK");
            break;
          }else if(temp == "count"){
            NUM_SAMPLES = input.substring(++i).toInt();
            // Serial.println(NUM_SAMPLES);
            sprintf(oTopic, "esl/%d/out/status", nodeID);
            client.publish(oTopic,"#OK");
            break;
          }
        }
      }
    }
  }
}

// mqtt callback when msg arrive
void callback(const MQTT::Publish& pub) {
  if (pub.has_stream()) {
    int read;
    while (read = pub.payload_stream()->read((uint8_t *)line_buf, LINE_BUF_SIZE)) {
      // Serial.write(line_buf, read);
    }
    pub.payload_stream()->stop();
    // Serial.println("");
  } else{
    // Serial.println(pub.topic());
    if(pub.topic() == "esl/check"){
      sprintf(oTopic, "esl/%d/out/status", nodeID);
      client.publish(oTopic,"#ACK");
    }else{
        checkCmd(pub.payload_string());
    }
  }
}

// fucntion check btn press
// void trigger_isr( ) {
//   if (!ext_irq_detected) {
//     ext_irq_detected = true;
//   }
// }

static unsigned long T1;
static unsigned long T2;
static unsigned long dTime = 0;

// fucntion data to broker เร็วเท่าไร?
void sendData() {
  voltageSum = INA219_getBusVoltage();          // 1   mV step
  currentSum = INA219_getCurrent() / 100;       // 0.1 mA step
  powerSum = voltageSum * currentSum / 1000;    // 0.1 mW step

  int16_t currentI = (int16_t)(currentSum / 10);
  int16_t currentF    = (int16_t)(abs(currentSum) % 10);
  int16_t powerI   = (int16_t)(powerSum / 10);
  int16_t powerF      = (int16_t)(abs(powerSum) % 10);

  // voltageSum = 3300;
  // currentSum = 123.45;
  // sprintf( sbuf, "%3d.%d," );  // เพื่ม แก้ format ด้วย "{'ct':%04d,'mA':%3d.%d,'mV':%4d,mW':%4d.%d,'us':%lu\n}"
  // T1 = micros();
  sprintf( sbuf, "{\"ct\":%d,\"st\":%d,\"mA\":%d.%d,\"mV\":%d,\"mW\":%d.%d,\"us\":%lu}\n",
                    samples, states, currentI, currentF, voltageSum, powerI, powerF, dTime );
  T1 = micros();
  client.publish(oTopic,sbuf);
  T2 = micros();
  dTime = T2-T1;
  // client.publish("esl/1/out/data",sbuf);
}

// fucntion check btn press
void trigger_isr( ) {
  if (!ext_irq_detected) {
    ext_irq_detected = true;
    states = 0;
  }else{
    states = (states+1)%NUM_STATES;
  }
}

void setup() {
  // Serial.begin(921600);

  pinMode(extInterrupt, INPUT_PULLUP);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  digitalWrite(ledR, LOW);

  Wire.begin(sda, scl); // use I2C port
  IN219_config();
  Wire.setClock(400000); // init scl speed 400kHz
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    // Serial.println("WiFi connected");
  }

  // timer1_isr_init();
  // // timer1_attachInterrupt(reinterpret_cast<timercallback>(readINA));
  // timer1_enable(TIM_DIV16, TIM_EDGE, 1);    //TIM_DIV16 -> 5MHz = 5 ticks/us, TIM_DIV1 -> 80MHz = 80 ticks/us
  // timer1_write(10000);                       //call interrupt after ... tick

  digitalWrite(ledG, HIGH);
  digitalWrite(ledR, HIGH);
  delay(500);
  digitalWrite(ledG, LOW);
  digitalWrite(ledR, LOW);
  delay(100);
  digitalWrite(ledG, HIGH);
  digitalWrite(ledR, HIGH);
  delay(500);
  digitalWrite(ledG, LOW);
  digitalWrite(ledR, LOW);
}

void loop() {
  // check MQTT connection to broker
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      if (client.connect("arduinoClient")) {
	      client.set_callback(callback);
        sprintf(iTopic, "esl/%d/in/+", nodeID);
        client.subscribe(iTopic);
        // client.subscribe("esl/check");
	      // client.subscribe("esl/"+nodeID+"/in/cmd");
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
      sprintf(oTopic, "esl/%d/out/status", nodeID);
       if ( cmd == CMD_DISARM ) {
          state = ST_RST;
          detachInterrupt( extInterrupt ); // use EINT0 / D3 input pin on 32u4
          ext_irq_enabled  = false;
          ext_irq_detected = false;
          delay(5);
          // Serial.println("disarmed");
          // Serial.println("#DISARMED");
          client.publish(oTopic, "#DISARMED");
          // client.publish(NODE_ID+"/status","#DISARMED");
          digitalWrite(ledG, LOW);
        }else if ( !ext_irq_enabled ) {
           state = ST_ARMED;
           detachInterrupt(getProcessPin);
           detachInterrupt(rstPin);
           attachInterrupt( extInterrupt, trigger_isr, FALLING ); // use EINT0 / D3 input pin on 32u4
           attachInterrupt(rstPin, trigger_isr, FALLING);
           ext_irq_enabled  = true;
           ext_irq_detected = false;
           delay(5);
          // Serial.println("armed");
          // Serial.println("#ARMED");
          client.publish(oTopic,"#ARMED");
          // client.publish(NODE_ID+"/status","#ARMED");
          digitalWrite(ledG, HIGH);
       }

       if ( ext_irq_detected ) {
          state = ST_SAMPLING; // go to state ST_SAMPLING
          detachInterrupt( extInterrupt ); // use EINT0 / D3 input pin on 32u4
          // attachInterrupt(getProcessPin, getProcess, FALLING);
          // attachInterrupt(rstPin, resetProcess, FALLING);
          ext_irq_enabled  = false;
          ext_irq_detected = false;
          samples = 0;
          // Serial.println("sampling started");
          // Serial.println("#START");
          client.publish(oTopic,"#START");
          // client.publish(NODE_ID+"/status","#START");
          sprintf(oTopic, "esl/%d/out/data", nodeID);
          digitalWrite(ledG, LOW);
          ts = micros();
          start_ts = ts;
       }
       break;

    case ST_SAMPLING:
       if ( micros() - ts >= INTERVAL_USEC ) { // run every 4 msec.
         startTime = micros();
         sendData();
         ts += INTERVAL_USEC;
         samples++;
         if(samples % 100 == 0){
           digitalWrite(ledR, !digitalRead(ledR));
         }
         if ( samples >= NUM_SAMPLES ) {
            state = ST_READY; // go to state ST_READY
            delay(5);
            // Serial.println("sampling ended");
            // Serial.println("#END");
            sprintf(oTopic, "esl/%d/out/status", nodeID);
            client.publish(oTopic,"#END");
            // client.publish(NODE_ID+"/status","#END");
            digitalWrite(ledR, LOW);
         }
       }
       break;

    case ST_READY:
       state = ST_RST;
       cmd = CMD_NONE;
       delay(5);
      //  Serial.println("ready");
      //  Serial.println("#READY");
      client.publish(oTopic,"#READY");
      //  client.publish(NODE_ID+"/status","#READY");
       digitalWrite(ledG, HIGH);
       break;

    default:
       state = ST_RST;
       break;
  }
}
///////////////////////////////////////////////////////////////////////////////
