/**
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */

//#define TEENSY0 //U1
#define TEENSY1  //U2

#include <SimpleFOC.h>
#include <Wire.h>
#include "FingerPosition.h"
#include <string>
#include "MagneticSensorSoftI2C.h"
#include <SoftWire.h>

SoftWire SWire(18,19);
SoftWire SWire1(17,16);
SoftWire SWire2(25,24);

//finger position classes
Gauntl33t::FingerPosition* fingerPosition0;
Gauntl33t::FingerPosition* fingerPosition1;
Gauntl33t::FingerPosition* fingerPosition2;

// magnetic sensor instance - SPI
MagneticSensorSoftI2C sensor0 = MagneticSensorSoftI2C(AS5600_I2C);
MagneticSensorSoftI2C sensor1 = MagneticSensorSoftI2C(AS5600_I2C);
MagneticSensorSoftI2C sensor2 = MagneticSensorSoftI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor0 = BLDCMotor(7, 4.8, 147);
BLDCMotor motor1 = BLDCMotor(7, 4.8, 147);
BLDCMotor motor2 = BLDCMotor(7, 4.8, 147);

#ifdef TEENSY0
BLDCDriver3PWM driver0 = BLDCDriver3PWM(11, 10, 9, 8);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(7, 6, 5, 4);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(3, 2, 1, 0);
#endif

#ifdef TEENSY1
BLDCDriver3PWM driver0 = BLDCDriver3PWM(3, 2, 1, 0);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(7, 6, 5, 4);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(11, 10, 9, 8);
#endif

// voltage set point variable
float target_voltage = 0.2;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void MotorSensorSetup(BLDCMotor& aMotor, MagneticSensorSoftI2C& aSensor, BLDCDriver3PWM& aDriver, uint8_t aChannel)
{
  //MUXChannelSelect(aChannel);
  switch (aChannel)
  {
    case 0:
      //SWire = new SoftwareI2C();
      SWire.begin();
      aSensor.init(&SWire);
      break;
    case 1:
      //SWire1 = new SoftwareI2C();
      SWire1.begin();
      aSensor.init(&SWire1);
      break;
    case 2:
      //SWire2 = new SoftwareI2C();
      SWire2.begin();
      aSensor.init(&SWire2);
      break;
    default:
      return;
  }
  
  aMotor.linkSensor(&aSensor);

  // power supply voltage
  aDriver.voltage_power_supply = 10.0;
  aDriver.init();
  aMotor.linkDriver(&aDriver);

  // aligning voltage 
  aMotor.voltage_sensor_align = 5;
  // choose FOC modulation (optional)
  aMotor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  aMotor.controller = MotionControlType::torque;
  // comment out if not needed
  aMotor.useMonitoring(Serial);
  // initialize motor
  aMotor.init();
  // align sensor and start FOC
  aMotor.initFOC();

  switch (aChannel)
  {
    case 0:
      SWire.end();
      break;
    case 1:
      SWire1.end();
      break;
    case 2:
      SWire2.end();
      break;
    default:
      return;
  }
}

void setup() {
  //_delay(10000);
  // use monitoring with serial 
  Serial.begin(115200);
  
  MotorSensorSetup(motor0,sensor0,driver0,0);
  MotorSensorSetup(motor1,sensor1,driver1,1);
  MotorSensorSetup(motor2,sensor2,driver2,2);

  // add target command T
  //command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void MotorLoop(BLDCMotor& aMotor,MagneticSensorSoftI2C& aSensor, uint8_t aChannel)
{
  switch(aChannel)
  {
    case 0:
      SWire.begin();
      break;
    case 1:
      SWire1.begin();
      break;
    case 2:
      SWire2.begin();
      break;
    default:
      return;
  }
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  aMotor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  aMotor.move(target_voltage);
  
  auto angle = aSensor.getSensorAngle();
  //Serial.print(String(angle).c_str());

  Gauntl33t::FingerPosition* fingerPosition;
  switch(aChannel)
  {
    case 0:
      if(fingerPosition0 == NULL)
      {
        fingerPosition0 = new Gauntl33t::FingerPosition(angle);
      }
      fingerPosition = fingerPosition0;
      
      Serial.print("Motor0:");
      Serial.print(String(fingerPosition->GetFingerPosition()).c_str());
      Serial.print(",");
      break;
    case 1:
      if(fingerPosition1 == NULL)
      {
        fingerPosition1 = new Gauntl33t::FingerPosition(angle);
      }
      fingerPosition = fingerPosition1;
      Serial.print("Motor1:");
      Serial.print(String(fingerPosition->GetFingerPosition()).c_str());
      Serial.print(",");
      break;
    case 2:
      if(fingerPosition2 == NULL)
      {
        fingerPosition2 = new Gauntl33t::FingerPosition(angle);
      }
      fingerPosition = fingerPosition2;
      Serial.print("Motor2:");
      Serial.println(String(fingerPosition->GetFingerPosition()).c_str());
      break;
    default:
      fingerPosition = fingerPosition0;
  }
  if(fingerPosition == NULL)
  {
    Serial.print("error");
    return;
  }
  fingerPosition->SensorUpdatePos(angle);
  auto fingerPos = fingerPosition->GetFingerPosition();
  String position = "{";
  position += aChannel;
  position += ":";
  position += String(angle);
  position += "}\n";
  //Serial.write(position.c_str());
  switch(aChannel)
  {
    case 0:
      SWire.end();
      break;
    case 1:
      SWire1.end();
      break;
    case 2:
      SWire2.end();
      break;
    default:
      return;
  }
}

void loop() {

  MotorLoop(motor0,sensor0,0);
  MotorLoop(motor1,sensor1,1);
  MotorLoop(motor2,sensor2,2);
  
  // user communication
  command.run();
}