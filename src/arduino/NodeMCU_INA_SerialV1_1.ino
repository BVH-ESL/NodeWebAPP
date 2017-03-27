// Allocate ram into struct array
// read INA INTERVAL_USEC sample 5 times and find avg and save to array
// user timer for reading sensor module data
// #include <ESP8266WiFi.h>
// #include <PubSubClient.h>
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

int32_t INA219_getCurrent() { // in uA step
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_CURRENT, &value );
  return ((int16_t)value) * (int32_t)CURRENT_LSB;
}

int32_t INA219_getPower() { // Load Power = Current * Bus Voltage
  uint16_t value;
  INA219_readReg( INA219_I2C_ADDR, INA219_REG_POWER, &value);
  return (((int16_t)value) * (int32_t)POWER_LSB) / 1000;
}

#define LINE_BUF_SIZE    (32)
#define extInterrupt     (0)      //D2
#define MIN_INTERVAL_USEC   (2500UL)
#define MAX_INTERVAL_USEC   (50000UL)

// max data is 2250 data
#define NUM_BLOCKS             (8)
#define NUM_BLOCK_DATA_UNITS   (250)
#define NUM_DATA_UNITS_TOTAL   (NUM_BLOCKS * NUM_BLOCK_DATA_UNITS)
#define MAX_NUM_SAMPLES NUM_DATA_UNITS_TOTAL

typedef struct _data_unit {
  uint16_t cnt;
  uint8_t  state;
  int16_t  volt;
  int16_t  current;
  uint16_t ts;
} data_unit_t;

static data_unit_t *buf[ NUM_BLOCKS ][ NUM_BLOCK_DATA_UNITS ];

int NUM_SAMPLES = NUM_DATA_UNITS_TOTAL;
unsigned long INTERVAL_USEC = 2500;

// define GPIO variable
#define scl              (5)                      // D1 pin
#define sda              (4)                       // D2 pin
#define ledR             (13)                       // D7
#define ledG             (15)                       // D8
#define getProcessPin    (12)           // D6 pin
#define rstPin           (14)                   // D5 pin

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

#define NUM_SAMPLES_AVG (5)
int8_t countSamplesAVG = 0;
int32_t currentSum = 0;
int32_t voltageSum = 0;

void readINAData(){
  start_ts = micros();
  voltageSum += INA219_getBusVoltage();
  currentSum += INA219_getCurrent() / 100;
  if(countSamplesAVG == NUM_SAMPLES_AVG-1){
    int32_t currentAVG = currentSum/NUM_SAMPLES_AVG;
    int32_t voltageAVG = voltageSum/NUM_SAMPLES_AVG;
    // power = currentAVG * currentAVG / 1000;

    // for real data
    data_unit_t *p;
    p = buf[samples/NUM_BLOCK_DATA_UNITS][samples%NUM_BLOCK_DATA_UNITS];
    p->cnt     = (uint16_t)samples;
    p->state   = (uint8_t)state;
    p->volt    = (int16_t)currentAVG;
    p->current = (int16_t)voltageAVG;
    p->ts      = (uint16_t)micros()-start_ts;

    // for mock data
    // data_unit_t *p;
    // p = buf[samples/NUM_BLOCK_DATA_UNITS][samples%NUM_BLOCK_DATA_UNITS];
    // p->cnt     = (uint16_t)samples;
    // p->state   = (uint8_t)random(0,5);
    // p->volt    = (int16_t)random(4500,5500);
    // p->current = (int16_t)random(0,500);
    // p->ts      = (uint16_t)random(2500,3500);
    // sprintf( buf[samples/NUM_BLOCK_MESSAGES][samples%NUM_BLOCK_MESSAGES], "{\"ct\":%d,\"st\":%d,\"mA\":%d.%d,\"mV\":%d,\"mW\":%d.%d,\"us\":%lu}",
    //                   samples, states, currentI, currentF, voltageAVG, powerI, powerF, micros()-start_ts );
    voltageSum = 0;
    currentSum = 0;
    samples++;
  }
  countSamplesAVG = (countSamplesAVG+1)%NUM_SAMPLES_AVG;
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
  Serial.begin(115200);
  Wire.begin(sda, scl); // use I2C port
  Wire.setClock(400000); // init scl speed 400kHz
  IN219_config();
  // Serial1.begin(500000); // MPU-MCU via serial
  // allcocate ram for struck array
  for ( int j=0; j < NUM_BLOCKS; j++ ) {
    for ( int i=0; i < NUM_BLOCK_DATA_UNITS; i++ ) {
       buf[j][i] = (data_unit_t *)malloc( sizeof(data_unit_t) );
    }
  }
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
          timer1_isr_init();
          timer1_attachInterrupt(reinterpret_cast<timercallback>(readINAData));
          timer1_enable(TIM_DIV16, TIM_EDGE, 1);    //TIM_DIV16 -> 5MHz = 5 ticks/us, TIM_DIV1 -> 80MHz = 80 ticks/us
          timer1_write(12500);                       //call interrupt after ... tick
       }
       break;

    case ST_SAMPLING:
       if ( samples >= NUM_SAMPLES ) {
          state = ST_READY; // go to state ST_READY
          delay(5);
          // Serial.println("sampling ended");
          Serial.println("#END");
          timer1_disable();
          data_unit_t *p;
          char publishBuf[64];
          for (int i=0; i < NUM_DATA_UNITS_TOTAL; i++) {
             p = buf[i/NUM_BLOCK_DATA_UNITS][i%NUM_BLOCK_DATA_UNITS];
             int32_t power = (p->volt * p->current)/1000;
             int16_t currentI = (int16_t)(p->current / 10);
             int16_t currentF    = (int16_t)(abs(p->current ) % 10);
             int16_t powerI   = (int16_t)(power / 10);
             int16_t powerF      = (int16_t)(abs(power) % 10);
            //  Serial.printf( "%u,%d,%d,%d,%ld\n", p->cnt, p->volt, p->current, (p->volt * p->current)/1000, p->ts );
             sprintf( publishBuf, "{\"ct\":%d,\"st\":%d,\"mA\":%d.%d,\"mV\":%d,\"mW\":%d.%d,\"us\":%lu}",
                               p->cnt, p->state, currentI, currentF, p->volt, powerI, powerF, p->ts );
            //  client.publish(oTopic,publishBuf);
            Serial.println(publishBuf);
             delay(5);
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
