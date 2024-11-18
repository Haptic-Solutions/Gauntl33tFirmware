/**
 * Gauntl33t Haptic Glove Firmware.
 * Based off of voltage_control.ino example for SimpleFOC
 */

/**Micro Controller Defines
 * Uncomment 1 Microcontroller
 * If using 2 Teensies uncomment TEENSY0 when programming one Teensy and uncomment TEENSY1 when programming the other Teensy.
 * If using Teensy Micromod uncomment MICROMOD
 * TODO add STM32_0 and STM32_1 for new boards
 */

//#define TEENSY0 //U1
//#define TEENSY1  //U2
#define MICROMOD

/**
 * Motor Define. Uncomment 1 line depending on motor type
 * IFLIGHT - Iflight IPower 2804
 * GOPRO - 2204 260KV
 * YUNEEC - YUNEEC 1613S (Not recommended. Get Hot.)
 */

#define IFLIGHT
//#define GOPRO
//#define YUNEEC

/**
 * Set Power Supply Voltage here.
 */
const float POWERSUPPLYVOLTAGE = 20.0f;

/**
 * Set the Max and Min target voltages such that you do not draw too much current.
 * If you are feeding in 10 volts you can set the max closer to 10.
 * If you are feeding in a higher voltage such as 20 volts you may want to set
 * this value closer to 1.
 */
const float MAXTARGETVOLTAGE = 1.0f;
const float MINTARGETVOLTAGE = 0.0f;

/**
 * Include Libraries.
 */
#include <iostream>
#include <unordered_map>
#include <SimpleFOC.h>
#include <Wire.h>
#include "FingerPosition.h"
#include <string>
#include "MagneticSensorSoftI2C.h"
#include <ArduinoJson.h>
#include <SoftWire.h>

/**
 * Create dictonaries of SoftWires to be used for sensors.
 * For some microconrollers you may be able to switch back to regular Wires if prefered.
 */
#if defined(TEENSY0) || defined(TEENSY1)
std::unordered_map<uint8_t,SoftWire*> sWires(
  {
    {0,new SoftWire(18,19)},
    {1,new SoftWire(17,16)},
   // {2,new SoftWire(25,24)}});
   {2,new SoftWire(25,29)}});
#endif

#ifdef MICROMOD
std::unordered_map<uint8_t,SoftWire*> sWires(
  {
    {0,new SoftWire(18,19)},
    {1,new SoftWire(25,24)},
    {2,new SoftWire(38,39)},
    {3,new SoftWire(41,40)},
    {4,new SoftWire(37,36)},
    {5,new SoftWire(35,34)}
    });
#endif

/**
 * Finger position classes for keeping track of finger movement.
 */
std::unordered_map<uint8_t,Gauntl33t::FingerPosition*> fingerPositions;

/**
 * Define number of motors based on microcontroller.
 */

#if defined(TEENSY0) || defined(TEENSY1)
const uint8_t NUMBER_MOTORS = 3;
#endif
#ifdef MICROMOD
const uint8_t NUMBER_MOTORS = 6;
#endif

/**
 * Map of I2C Magnetic Sensors.
 */
std::unordered_map<uint8_t,MagneticSensorSoftI2C*> sensorx;

/**
 * Map of Motors.
 */
std::unordered_map<uint8_t, BLDCMotor*> motorx;

/**
 * Define Map of Drivers based on microcontroller.
 */
#ifdef TEENSY0
std::unordered_map<uint8_t,BLDCDriver3PWM*> driverx(
  {
    {0, new BLDCDriver3PWM(11, 10, 9, 8)},
    {1, new BLDCDriver3PWM(7, 6, 5, 4)},
    {2, new BLDCDriver3PWM(3, 2, 1, 0)}
  });
#endif

#ifdef TEENSY1
std::unordered_map<uint8_t,BLDCDriver3PWM*> driverx(
  {
    {0, new BLDCDriver3PWM(3, 2, 1, 0)},
    {1, new BLDCDriver3PWM(7, 6, 5, 4)},
    {2, new BLDCDriver3PWM(11, 10, 9, 8)}
  });
#endif

#ifdef MICROMOD
std::unordered_map<uint8_t,BLDCDriver3PWM*> driverx(
  {
    {0, new BLDCDriver3PWM(23, 7, 8, 42)},
    {1, new BLDCDriver3PWM(15, 14, 3, 43)},
    {2, new BLDCDriver3PWM(5, 29, 4, 44)},
    {3, new BLDCDriver3PWM(33, 9, 6, 45)},
    {4, new BLDCDriver3PWM(13, 11, 12, 26)},
    {5, new BLDCDriver3PWM(1, 2, 10, 32)}
  });
#endif

/**
 * Set target Voltages for each motor
 */
float target_voltages[6] = {0.2f,0.2f,0.2f,0.2f,0.2f,0.2f};

/**
 * Array for tracking time between frames sent for each finger.
 * Helps to set message frequency to avoid too much network congestion.
 */
elapsedMillis fingerTimes[6] = {0,0,0,0,0,0};

/**
 * Function used to setup the Motors and Sensors
 * @Param aMotor to be setup.
 * @Param aSensor to be setup.
 * @Param aDriver to be setup.
 * @Param aChannel to be setup.
 */
void MotorSensorSetup(BLDCMotor* aMotor, MagneticSensorSoftI2C* aSensor, BLDCDriver3PWM* aDriver, uint8_t aChannel)
{
  //Check if channel exists before proceeding. If not return.
  if(sWires.find(aChannel) == sWires.end())
  {
    return;
  }

  //Call begin function for appropriate soft wire i2c channel.
  sWires[aChannel]->begin();
  //Initialize sensor with soft wire i2c channel.
  aSensor->init(sWires[aChannel]);
  //Link motor to sensor.
  aMotor->linkSensor(aSensor);

  //Set Power supply voltage
  aDriver->voltage_power_supply = POWERSUPPLYVOLTAGE;
  //Initialize motor driver
  aDriver->init();
  //Link driver to motor
  aMotor->linkDriver(aDriver);

  // Set the voltage to be used while aligning the motors
  aMotor->voltage_sensor_align = 1.0f;
  // Choose FOC modulation (optional)
  aMotor->foc_modulation = FOCModulationType::SpaceVectorPWM;
  // Set motion control loop to be used.
  aMotor->controller = MotionControlType::torque;
  // Uncomment for more SimpleFOC debugging.
  //aMotor.useMonitoring(Serial);
  // Initialize the motor.
  aMotor->init();
  // Align the sensor and start FOC.
  aMotor->initFOC();
  // Call end on soft wire i2c channel.
  sWires[aChannel]->end();
}

/**
 * setup function.
 * Initializes everything before the main loop.
 */
void setup() {
  //Setup motors and sensors based on motor type.
  for(int i = 0; i < NUMBER_MOTORS; i++)
  {
    sensorx[i] = new MagneticSensorSoftI2C(AS5600_I2C);
    #ifdef IFLIGHT
    motorx[i] = new BLDCMotor(7, 4.8, 147);
    #endif
    #ifdef GOPRO
    motorx[i] = new BLDCMotor(7, 4.8, 260);
    #endif
    #ifdef YUNEEC
    motorx[i] = new BLDCMotor(5, 5.55, 610);
    #endif
  }
  //Setup Serial2 for communication.
  Serial2.begin(115200);

  //Call MotorSensorSetup function to complete setup on motors and sensors.
  for(uint8_t i = 0; i < NUMBER_MOTORS; i++)
  {
    MotorSensorSetup(motorx[i],sensorx[i],driverx[i],i);
  }

  //Wait a second before continuing to main loop.
  _delay(1000);
}

/**
 * Performs motor portion of main loop
 * @param aMotor to be updated.
 * @param aSensor to be updated.
 * @param aChannel to be updated.
 */
void MotorLoop(BLDCMotor* aMotor,MagneticSensorSoftI2C* aSensor, uint8_t aChannel)
{
  //Check if channel exists. If not return.
  if(sWires.find(aChannel) == sWires.end())
  {
    return;
  }
  //Call begin on sWire I2C channel.
  sWires[aChannel]->begin();

  //Run the FOC Algorithm.
  aMotor->loopFOC();

  // If the target voltage is in an acceptable range.
  // Set the target voltage.
  if((target_voltages[aChannel] <= MAXTARGETVOLTAGE) && (target_voltages[aChannel] >= MINTARGETVOLTAGE ))
  {
    aMotor->move(target_voltages[aChannel]);
  }

  //Get the sensor angle.
  auto angle = aSensor->getSensorAngle();

  //Check if finger position exists for a motor. If not create it.
  if(fingerPositions.find(aChannel) == fingerPositions.end())
  {
    fingerPositions[aChannel] = new Gauntl33t::FingerPosition(angle);
  }

  //Update finger position with sensor angle.
  fingerPositions[aChannel]->SensorUpdatePos(angle);
  auto fingerPos = fingerPositions[aChannel]->GetFingerPosition();
  
  #if defined(TEENSY0) || defined(MICROMOD)
  auto tempChannel = aChannel;
  auto tempFingerPos = 1.0 - fingerPos;
  #endif
  #ifdef TEENSY1
  auto tempChannel = aChannel + 3;
  auto tempFingerPos = fingerPos;
  #endif

  //If it has been long enough since last finger position was sent.
  //Send finger position as a JSON style message.
  if(fingerTimes[aChannel]>100)
  {
    fingerTimes[aChannel]=0;
    String position = "{\"FP\":[[";
    position += tempChannel;
    position += ",";
    position += String(tempFingerPos);
    position += "]]}\n";
    Serial2.write(position.c_str());
  }
  //Call end on sWire i2c channel.
  sWires[aChannel]->end();
}

/**
 * JsonDoc for processing received Json messages.
 */
StaticJsonDocument<200> JsonDoc;

/**
 * Process JSON messages
 * @param COMMAND handle received COMMAND message.
 */
void jsonProcess(String COMMAND) 
{
  //Clear the previous JSON message.
  JsonDoc.clear();
  //Read in new JSON command.
  auto JsonError = deserializeJson(JsonDoc, COMMAND.c_str());
  //Handle error case.
  if (JsonError) {
    //todo add errors
  } 
  //Get and set target voltages for all fingers.
  else
  {
    if (JsonDoc["TV"].as<String>().length() >= 2 && JsonDoc["TV"].as<String>() != "null") { 
      for (JsonVariant value : JsonDoc["TV"].as<JsonArray>()) {
        if (value.as<JsonArray>()) {
          int FingerNumber = value[0].as<int>();
          #ifdef TEENSY0
            if(FingerNumber > 2)
            {
              return;
            }
          #endif
          #ifdef TEENSY1
            if(FingerNumber < 3)
            {
              return;
            }
            FingerNumber = FingerNumber -3;
          #endif
          if((FingerNumber < 0) || (FingerNumber>5))
          {
            return;
          }
          
          target_voltages[FingerNumber] = value[1].as<float>();

          if(target_voltages[FingerNumber] > 1.0f)
          {
            target_voltages[FingerNumber] = 1.0f;
          }
          if(target_voltages[FingerNumber] < 0.0f)
          {
            target_voltages[FingerNumber] = 0.0f;
          }
        }
      }
    }
  }
}

/**
 * main loop function. Call sub loops for handling motors and JSON messages.
 */
void loop() 
{
  //Update all motors and read all positions to send finger positions.
  for(uint8_t i = 0; i < NUMBER_MOTORS; i++)
  {
    MotorLoop(motorx[i],sensorx[i],i);
  }
  //Read in target voltages for all motors.
  if (Serial2.available()) {
    auto buffer =Serial2.readStringUntil('\n');
    jsonProcess(buffer);
  }
}
