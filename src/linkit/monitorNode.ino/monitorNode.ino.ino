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

#define LINE_BUF_SIZE    (32)
#define extInterrupt     (0)      //D2
#define MIN_INTERVAL_USEC   (2500UL)
#define MAX_INTERVAL_USEC   (50000UL)

int NUM_SAMPLES = 500;
unsigned long INTERVAL_USEC = 2500;

char sbuf[64];
bool setTime = false;
bool setCount = false;
uint16_t samples = 0;

char line_buf[ LINE_BUF_SIZE + 1 ];
uint8_t line_buf_index = 0;
uint32_t ts, start_ts;

enum states {
  ST_NONE = 0, ST_RST, ST_ARMED, ST_SAMPLING, ST_READY
};

enum commands {
  CMD_NONE = 0, CMD_ARM, CMD_DISARM
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
  uint16_t a0 = analogRead(A0);
  uint16_t a1 = analogRead(A1);
  delayMicroseconds(2000);
  sprintf( sbuf, "%04d,%u,%u,%lu\n", samples, a0, a1, micros() );
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
      else if ( !strcmp(line_buf, "CHECK") ) {
        Serial1.println("#ACK");
      }
      else if(setTime){
        // convert line_buf to int
        setTime = false;
      }else if(setCount){
        // convert line_buf to int
        setCount = false;
      }
    }
    else if (ch == ',') {
      line_buf[ line_buf_index ] = '\0';
      line_buf_index = 0;
      if ( !strcmp(line_buf, "time") ) {
        setTime = true;
        setCount = false;
      }
      else if ( !strcmp(line_buf, "count") ) {
        setTime = false;
        setCount = true;
      }
    }
  }
  return cmd;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(500000); // MPU-MCU via serial
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
        detachInterrupt( 0 ); // use EINT0 / D3 input pin on 32u4
        ext_irq_enabled  = false;
        ext_irq_detected = false;
        delay(5);
        Serial.println("disarmed");
        Serial1.println("#DISARMED");
      }
      else if ( !ext_irq_enabled ) {
        state = ST_ARMED;
        attachInterrupt( 0, trigger_isr, FALLING ); // use EINT0 / D3 input pin on 32u4
        ext_irq_enabled  = true;
        ext_irq_detected = false;
        delay(5);
        Serial.println("armed");
        Serial1.println("#ARMED");
      }

      if ( ext_irq_detected ) {
        state = ST_SAMPLING; // go to state ST_SAMPLING
        detachInterrupt( 0 ); // use EINT0 / D3 input pin on 32u4
        ext_irq_enabled  = false;
        ext_irq_detected = false;
        samples = 0;
        Serial.println("sampling started");
        Serial1.println("#START");
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
          Serial.println("sampling ended");
          Serial1.println("#END");
        }
      }
      break;

    case ST_READY:
      state = ST_RST;
      delay(5);
      Serial.println("ready");
      Serial1.println("#READY");
      break;

    default:
      state = ST_RST;
      break;
  }
}
