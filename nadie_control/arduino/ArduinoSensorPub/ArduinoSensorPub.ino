#include <ros.h>
#include <std_msgs/String.h>
#include "nadie_control/ArduinoSensors.h"
#include <i2c_t3.h>
#include <VL53L0X.h>
#include "TMP102.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

#define SENSOR_COUNT 4
#define SENSOR_LOOP_COUNT 4

int XSHUT[SENSOR_COUNT] = {4, 5, 6, 7};
int SENSOR_I2C_ADDRESS[SENSOR_COUNT] = {61, 62, 63, 64};
VL53L0X sensor[SENSOR_COUNT];

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE1_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_37_38, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

ros::NodeHandle  nh;

nadie_control::ArduinoSensors arduinoSensors;
ros::Publisher pub("arduino_sensors", &arduinoSensors);

TMP102 sensor48(0x48); // Initialize sensor at I2C address 0x48
TMP102 sensor49(0x49);

void initTemperatureSensor(TMP102& sensor) {
  sensor.begin();  // Join I2C bus

  // Initialize sensor settings
  // These settings are saved in the sensor, even if it loses power
  
  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor.setFault(0);  // Trigger alarm immediately
  
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor.setAlertPolarity(1); // Active HIGH
  
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor.setAlertMode(0); // Comparator Mode.
  
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor.setConversionRate(2);
  
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor.setExtendedMode(0);

  //set T_HIGH, the upper limit to trigger the alert on
  sensor.setHighTempF(125.0);  // set T_HIGH in F
  //sensor.setHighTempC(29.4); // set T_HIGH in C
  
  //set T_LOW, the lower limit to shut turn off the alert
  sensor.setLowTempF(84.0);  // set T_LOW in F
  //sensor.setLowTempC(26.67); // set T_LOW in C
}

void setup() { 
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  bno.begin();
  delay(1000);


  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  
  
  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    digitalWrite(XSHUT[i], HIGH);
    delay(10);
    sensor[i].setAddress(SENSOR_I2C_ADDRESS[i]);
    delay(10);
    sensor[i].init();
    sensor[i].setTimeout(200);
    sensor[i].startContinuous(0);
    sensor[i].setMeasurementTimingBudget(20000);
  }

  initTemperatureSensor(sensor48);
  initTemperatureSensor(sensor49);
  
  // Turn sensor on to start temperature measurement.
  // Current consumtion typically ~10uA.
  sensor48.wakeup();
  sensor49.wakeup();

}

void loop()
{
//  delay(200);
  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    arduinoSensors.frontLeftMm = sensor[0].readRangeContinuousMillimeters();
    arduinoSensors.frontRightMm = sensor[1].readRangeContinuousMillimeters();
    arduinoSensors.backLeftMm = sensor[2].readRangeContinuousMillimeters();
    arduinoSensors.backRightMm = sensor[3].readRangeContinuousMillimeters();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    arduinoSensors.euler_x = euler.x();
    arduinoSensors.euler_y = euler.y();
    arduinoSensors.euler_z = euler.z();
  }

  // read temperature data
  arduinoSensors.motor1_temp = sensor48.readTempF();
  arduinoSensors.motor2_temp = sensor49.readTempF();

  pub.publish(&arduinoSensors);
  nh.spinOnce();
}


