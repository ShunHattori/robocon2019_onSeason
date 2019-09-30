/*
powered via mbed 3.3v, no additional pullups on I2C necessary]
*/
 
#ifndef LidarLite_H
#define LidarLite_H 
 
#include <mbed.h>

// Default I2C Address of LIDAR-Lite.
#define LIDARLite_WriteAdr  0xc4    //8-bit slave write address
#define LIDARLite_ReadAdr   0xc5    //8-bit slave read address

// Commands
#define SET_CommandReg       0x00   // Register to write to initiate ranging
#define AcqMode              0x04   // Value to set in control register to initiate ranging

// Read Registers
#define GET_DistanceHBReg    0x0f   // High byte of distance reading data
#define GET_DistanceLBReg    0x10   // Low byte of distance reading data
#define GET_Distance2BReg    0x8f   // Register to get both High and Low bytes of distance reading data in 1 call
#define GET_VelocityReg      0x09   // Velocity measutement data


class LidarLite
{
    public:
        LidarLite(PinName sda, PinName scl);    //Constructor
        LidarLite(PinName sda, PinName scl, char currentAddress);    //Constructor with current address
        void refreshRangeVelocity();            //refreshes range and velocity data from registers of sensor
        void refreshRangeVelocityAdd(char currentAddres); //refreshes range and velocity data from registers of sensor
        void refreshVelocity();                 //refreshes velocity data from registers of sensor
        void refreshVelocityAdd(char currentAddres); //refreshes velocity data from registers of sensor
        void refreshRange();                    //refreshes range data from registers of sensor
        void refreshRangeAdd(char currentAddres);
        void changeAddress(char currentAddres,char newAddress);
        
        int16_t getRange_cm();                  //Read distance in cm from sensor
        int16_t getVelocity_cms();              //Read velocity in cm/s from sensor
        
    private:
        I2C* i2c_;
        int16_t distance_LL;
        int16_t velocity_LL;
};

#endif /* LidarLite_H */