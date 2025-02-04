//Modified MagneticSensorI2C.h to use SoftWire Library
#ifndef MAGNETICSENSORSOFTI2C_LIB_H
#define MAGNETICSENSORSOFTI2C_LIB_H

#include "SoftWire.h"
#include <SimpleFOC.h>


class MagneticSensorSoftI2C: public Sensor{
 public:
    /**
     * MagneticSensorI2C class constructor
     * @param chip_address  I2C chip address
     * @param bits number of bits of the sensor resolution 
     * @param angle_register_msb  angle read register msb
     * @param _bits_used_msb number of used bits in msb
     */
    MagneticSensorSoftI2C(uint8_t _chip_address, int _bit_resolution, uint8_t _angle_register_msb, int _msb_bits_used);

    /**
     * MagneticSensorI2C class constructor
     * @param config  I2C config
     */
    MagneticSensorSoftI2C(MagneticSensorI2CConfig_s config);
        
    /** sensor initialise pins */
    void init(SoftWire* _wire = new SoftWire(18,19));

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    /** experimental function to check and fix SDA locked LOW issues */
    int checkBus(byte sda_pin , byte scl_pin );

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    uint16_t lsb_used; //!< Number of bits used in LSB register
    uint8_t lsb_mask;
    uint8_t msb_mask;
    
    // I2C variables
    uint8_t angle_register_msb; //!< I2C angle register to read
    uint8_t chip_address; //!< I2C chip select pins

    // I2C functions
    /** Read one I2C register value */
    int read(uint8_t angle_register_msb);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
    
    /* the two wire instance for this sensor */
    SoftWire* wire;
    char swTxBuffer[16];
    char swRxBuffer[16];
};


#endif
