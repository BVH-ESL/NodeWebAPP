#include <Arduino.h>
#include <Wire.h>

/*********************************************************************
   Shunt Voltage = the voltage between V+ and V-.
   Bus Voltage   = the voltage between V- and GND, and it is the difference
                   between the Supply Voltage and the Shunt Voltage.

 R_SHUNT = 0.1 ohm (1%, 2W rating)
 Select V_BUS_MAX = 16V
 Select V_SHUNT_MAX = 0.08V or 80mV (select PGA=2)
 => I_MAX = V_SHUNT_MAX / R_SHUNT = 800mA = 0.8A
 Select I_MAX(expected) = 0.5A (less than I_MAX)

 Calculate the Current LSB
   15-bit: I_MAX/32768 = 0.5A/32767 =  15.259 uA per bit
   12-bit: I_MAX/4096  = 0.5A/4096  = 122.070 uA per bit
 Select LSB value(between I_MAX/32767 and I_MAX/4096)
   => choose LSB = 20uA per bit = 0.02mA per bit

 Calculate the Calibration value
   cal_val = int( 4096 x 10^-5 / (LSB * R_SHUNT) )
           = int( 4096 x 10^-5 / (20 * 10^-6 * 10^-1) )
           = int( 4096 x 5 )
           = 20480

 => Max current = 20uA/bit * 32767 = 655.4 mA
 => Max shunt voltage = 655.4mA * 0.1 Ohm = 65.5mV < (less than V_SHUNT_MAX=80mV)
 => Max power (for load) = 0.655A * 16V = 10.48W (assume V_BUS = V_BUS_MAX)

 Calculate power LSB
   LSB = 20 * Current_LSB
       = 20 * 20 x 10^-6 = 400uW per bit

**********************************************************************/

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


// set testing variable
#define debugPin 14           // D5 pin
#define extInterrupt     (0)      //D2
// #define pubProcessPin 12            //D6 pin
// #define startProcessPin  13         // D7 pin
// #define enablePin 15                // D8 pin
#define scl 5                       // D1 pin
#define sda 4                       // D2 pin
char sbuf[32];
int count = 0;

int32_t currentSum = 0;
int32_t currentMax = 0;
int32_t currentMin = 500;
int32_t voltageSum = 0;
int32_t voltageMax = 0;
int32_t voltageMin = 5000;
int32_t voltageShunt = 0;
int32_t voltageShuntMax = 0;
int32_t voltageShuntMin = 500;

volatile boolean ext_irq_detected = false;
//init timer1 function type and variable
typedef void (*timercallback)(void);

void readINA(){
  // detachInterrupt( extInterrupt );
  int32_t value = 0;
  value = INA219_getBusVoltage();
  value = INA219_getCurrent() / 100;
  // voltageSum += value;
  // voltageMax = _max(value, voltageMax);
  // voltageMin = _min(value, voltageMin);
  // // sprintf( sbuf, " %4dmV,\t", value );
  // // Serial.print(sbuf);
  //
  // // value = INA219_getShuntVoltage()/10;
  // // voltageShunt += value;
  // // voltageShuntMax = _max(value, voltageShuntMax);
  // // voltageShuntMin = _min(value, voltageShuntMin);
  // // sprintf( sbuf, " %3d.%02d mV\t", (int16_t)(value/100), (int16_t)(abs(value)%100) );
  // // Serial.print( sbuf );
  //
  // value = INA219_getCurrent() / 100;
  // currentSum += value;
  // currentMax = _max(value, currentMax);
  // currentMin = _min(value, currentMin);
  // sprintf( sbuf, " %3d.%dmA,\t", (int16_t)(value / 10), (int16_t)(abs(value) % 10) );
  // Serial.println(sbuf);

  count = (count+1)%100;
  // if(count == 99){
  //   currentSum /= count;
  //   voltageSum /= count;
  //   voltageShunt /= count;
  //   Serial.print("\n--------------sumary result-----------------\n");
  //   Serial.print("Bus voltage\tSense Current\n");
  //   sprintf( sbuf, " %4dmV,\t", voltageSum );
  //   Serial.print(sbuf);
  //   // sprintf( sbuf, " %3d.%02d mV\t", (int16_t)(voltageShunt/100), (int16_t)(abs(voltageShunt)%100) );
  //   // Serial.print( sbuf );
  //   sprintf( sbuf, " %3d.%dmA,\t", (int16_t)(currentSum / 10), (int16_t)(abs(currentSum) % 10) );
  //   Serial.print(sbuf);
  //   Serial.println("AVG");
  //
  //   sprintf( sbuf, " %4dmV,\t", voltageMin );
  //   Serial.print(sbuf);
  //   // sprintf( sbuf, " %3d.%02d mV\t", (int16_t)(voltageShuntMin/100), (int16_t)(abs(voltageShuntMin)%100) );
  //   // Serial.print( sbuf );
  //   sprintf( sbuf, " %3d.%dmA,\t", (int16_t)(currentMin / 10), (int16_t)(abs(currentMin) % 10) );
  //   Serial.print(sbuf);
  //   Serial.println("MIN");
  //
  //   sprintf( sbuf, " %4dmV,\t", voltageMax );
  //   Serial.print(sbuf);
  //   // sprintf( sbuf, " %3d.%02d mV\t", (int16_t)(voltageShuntMax/100), (int16_t)(abs(voltageShuntMax)%100) );
  //   // Serial.print( sbuf );
  //   sprintf( sbuf, " %3d.%dmA,\t", (int16_t)(currentMax / 10), (int16_t)(abs(currentMax) % 10) );
  //   Serial.print(sbuf);
  //   Serial.println("MAX");
  //   voltageSum = 0;
  //   currentSum = 0;
  //   // timer1_disable();
  //   ext_irq_detected = false;
  //   // attachInterrupt( extInterrupt, trigger_isr, FALLING );
  // }
}

// fucntion check btn press
void trigger_isr( ) {
  if (!ext_irq_detected) {
    // Serial.println("start");
    // Serial.print("\n-------------start Testing------------------\n");
    // Serial.print("Bus voltage\tShunt voltage\tSense Current\n");
    ext_irq_detected = true;
    // int32_t currentSum = 0;
    currentMax = 0;
    currentMin = 5000;
    // int32_t voltageSum = 0;
    voltageMax = 0;
    voltageMin = 5000;
    // int32_t voltageShunt = 0;
    voltageShuntMax = 0;
    voltageShuntMin = 5000;
  }
}

void setup() {
  Serial.begin(115200); // use serial port
  // pinMode(debugPin, OUTPUT);
  // digitalWrite(debugPin, HIGH);
  pinMode(extInterrupt, INPUT_PULLUP);
  Wire.begin(sda, scl); // use I2C port
  IN219_config();
  Wire.setClock(450000);
  timer1_isr_init();
  timer1_attachInterrupt(reinterpret_cast<timercallback>(readINA));
  timer1_enable(TIM_DIV16, TIM_EDGE, 1);    //TIM_DIV16 -> 5MHz = 5 ticks/us, TIM_DIV1 -> 80MHz = 80 ticks/us
  timer1_write(25000);                       //call interrupt after ... tick
  // attachInterrupt( extInterrupt, trigger_isr, FALLING );
}

void loop(){

}
