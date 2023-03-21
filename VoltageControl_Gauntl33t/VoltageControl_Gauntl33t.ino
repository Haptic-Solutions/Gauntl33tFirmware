/**
 * Torque control example using voltage control loop.
 * 
 * Most of the low-end BLDC driver boards doesn't have current measurement therefore SimpleFOC offers 
 * you a way to control motor torque by setting the voltage to the motor instead hte current. 
 * 
 * This makes the BLDC motor effectively a DC motor, and you can use it in a same way.
 */

//#define TEENSY0 //U1
//#define TEENSY1  //U2
#define MICROMOD
#define IFLIGHT
//#define GOPRO
//#define YUNEEC

#include <iostream>
#include <unordered_map>
#include <SimpleFOC.h>
#include <Wire.h>
#include "FingerPosition.h"
#include <string>
#include "MagneticSensorSoftI2C.h"
#include <ArduinoJson.h>

#include <SoftWire.h>

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

//finger position classes
std::unordered_map<uint8_t,Gauntl33t::FingerPosition*> fingerPositions;

#if defined(TEENSY0) || defined(TEENSY1)
const uint8_t NUMBER_MOTORS = 3;
#endif
#ifdef MICROMOD
const uint8_t NUMBER_MOTORS = 6;
#endif

// magnetic sensor instance - SPI
std::unordered_map<uint8_t,MagneticSensorSoftI2C*> sensorx;

// BLDC motor & driver instance
std::unordered_map<uint8_t, BLDCMotor*> motorx;

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

// voltage set point variable
float target_voltages[6] = {0.2f,0.2f,0.2f,0.2f,0.2f,0.2f};

elapsedMillis fingerTimes[6] = {0,0,0,0,0,0};

void MotorSensorSetup(BLDCMotor* aMotor, MagneticSensorSoftI2C* aSensor, BLDCDriver3PWM* aDriver, uint8_t aChannel)
{
  if(sWires.find(aChannel) == sWires.end())
  {
    return;
  }
  sWires[aChannel]->begin();
  aSensor->init(sWires[aChannel]);
  
  aMotor->linkSensor(aSensor);

  // power supply voltage
  aDriver->voltage_power_supply = 20.0f;
  aDriver->init();
  aMotor->linkDriver(aDriver);

  // aligning voltage 
  aMotor->voltage_sensor_align = 1.0f;
  // choose FOC modulation (optional)
  aMotor->foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  aMotor->controller = MotionControlType::torque;
  // comment out if not needed
  //aMotor.useMonitoring(Serial);
  // initialize motor
  aMotor->init();
  // align sensor and start FOC
  aMotor->initFOC();

  sWires[aChannel]->end();
}

void setup() {
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
  // use monitoring with serial 
  Serial2.begin(115200);
  
  for(uint8_t i = 0; i < NUMBER_MOTORS; i++)
  {
    MotorSensorSetup(motorx[i],sensorx[i],driverx[i],i);
  }

  //Serial2.println(F("Motor ready."));
  //Serial2.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void MotorLoop(BLDCMotor* aMotor,MagneticSensorSoftI2C* aSensor, uint8_t aChannel)
{
  if(sWires.find(aChannel) == sWires.end())
  {
    return;
  }
  sWires[aChannel]->begin();

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  aMotor->loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  if((target_voltages[aChannel] <= 1.0) && (target_voltages[aChannel]>=0.00))
  {
    aMotor->move(target_voltages[aChannel]);
  }
  
  auto angle = aSensor->getSensorAngle();

  if(fingerPositions.find(aChannel) == fingerPositions.end())
  {
    fingerPositions[aChannel] = new Gauntl33t::FingerPosition(angle);
  }
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
  sWires[aChannel]->end();
}

StaticJsonDocument<200> JsonDoc;
void jsonProcess(String COMMAND) 
{
  //Serial2.write("received\n");
  JsonDoc.clear();
  auto JsonError = deserializeJson(JsonDoc, COMMAND.c_str());
  if (JsonError) {
    //todo add errors
  } 
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

void loop() 
{
  for(uint8_t i = 0; i < NUMBER_MOTORS; i++)
  {
    MotorLoop(motorx[i],sensorx[i],i);
  }
  if (Serial2.available()) {
    auto buffer =Serial2.readStringUntil('\n');
    jsonProcess(buffer);
  }
}