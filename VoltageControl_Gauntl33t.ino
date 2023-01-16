/**
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */

#define TEENSY0 //U1
//#define TEENSY1  //U2

#include <SimpleFOC.h>
#include <Wire.h>
#include "FingerPosition.h"

//finger position classes
Gauntl33t::FingerPosition* fingerPosition0;
Gauntl33t::FingerPosition* fingerPosition1;
Gauntl33t::FingerPosition* fingerPosition2;

// magnetic sensor instance - SPI
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

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

void MotorSensorSetup(BLDCMotor& aMotor, MagneticSensorI2C& aSensor, BLDCDriver3PWM& aDriver, uint8_t aChannel)
{
  //MUXChannelSelect(aChannel);
  switch (aChannel)
  {
    case 0:
      Wire.setSCL(19);
      Wire.setSDA(18);
      Wire.begin();
      aSensor.init();
      break;
    case 1:
      Wire1.setSCL(16);
      Wire1.setSDA(17);
      Wire1.begin();
      aSensor.init(&Wire1);
      break;
    case 2:
      Wire2.setSCL(24);
      Wire2.setSDA(25);
      Wire2.begin();
      aSensor.init(&Wire2);
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
  //aMotor.useMonitoring(Serial);
  // initialize motor
  aMotor.init();
  // align sensor and start FOC
  aMotor.initFOC();

  switch (aChannel)
  {
    case 0:
      Wire.end();
      break;
    case 1:
      Wire1.end();
      break;
    case 2:
      Wire2.end();
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
  command.add('T', doTarget, "target voltage");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void MotorLoop(BLDCMotor& aMotor,MagneticSensorI2C& aSensor, uint8_t aChannel)
{
  switch(aChannel)
  {
    case 0:
      Wire.begin();
      break;
    case 1:
      Wire1.begin();
      break;
    case 2:
      Wire2.begin();
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

  Gauntl33t::FingerPosition* fingerPosition;
  switch(aChannel)
  {
    case 0:
      fingerPosition = fingerPosition0;
      break;
    case 1:
      fingerPosition = fingerPosition1;
      break;
    case 2:
      fingerPosition = fingerPosition2;
      break;
    default:
      fingerPosition = fingerPosition0;
  }
  if(fingerPosition == NULL)
  {
    fingerPosition = new Gauntl33t::FingerPosition(angle);
  }
  fingerPosition->SensorUpdatePos(angle);
  auto fingerPos = fingerPosition->GetFingerPosition();
  String position = "{";
  position += aChannel;
  position += ":";
  position += fingerPos;
  position += "}\n";
  Serial.write(position.c_str());
  switch(aChannel)
  {
    case 0:
      Wire.end();
      break;
    case 1:
      Wire1.end();
      break;
    case 2:
      Wire2.end();
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