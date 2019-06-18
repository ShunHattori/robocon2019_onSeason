/*
 *  Library for easy interface of LidarLite with mbed using I2C
 *  
 *  Akash Vibhute   <akash . roboticist [at] gmail . com>
 *  
 *  v0.1, 17/Feb/2015 - First version of library, tested using LPC1768 [powered via mbed 3.3v, no additional pullups on I2C necessary]
 *
 */
 
#include "LidarLite.h"

LidarLite::LidarLite(PinName sda, PinName scl)
{
    i2c_ = new I2C(sda, scl);
    i2c_->frequency(400000); //I2C @ 100kHz
    wait(0.5);
}
LidarLite::LidarLite(PinName sda, PinName scl, char currentAddress)
{
    i2c_ = new I2C(sda, scl);
    i2c_->frequency(400000); //I2C @ 100kHz
    wait(0.5);    
}  

int16_t LidarLite::getRange_cm()
{
   return(distance_LL);
}

int16_t LidarLite::getVelocity_cms()
{
    if(velocity_LL < 127)
        return(velocity_LL*10);
    else
        return((velocity_LL-256)*10);
}

void LidarLite::refreshRangeAdd(char currentAddres)
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};//00,0x04
    char read_dist[1]={GET_Distance2BReg};//0x8f
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        //nackack = i2c_->write(LIDARLite_WriteAdr, write, 2);
        nackack = i2c_->write((currentAddres<<1), write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
//        nackack = i2c_->write(LIDARLite_WriteAdr, read_dist, 1);
        nackack = i2c_->write((currentAddres<<1), read_dist, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
            nackack = i2c_->read(((currentAddres<<1)|0X01), dist, 2);
//        nackack = i2c_->read(LIDARLite_ReadAdr, dist, 2);
    }
    distance_LL = ((uint16_t)dist[0] << 8) + (uint16_t)dist[1];
}
void LidarLite::refreshRange()
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};//00,0x04
    char read_dist[1]={GET_Distance2BReg};//0x8f
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, read_dist, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(LIDARLite_ReadAdr, dist, 2);
    }
    distance_LL = ((uint16_t)dist[0] << 8) + (uint16_t)dist[1];
}
void LidarLite::refreshVelocity()
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};
    char read_dist[1]={GET_Distance2BReg};
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, read_vel, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(LIDARLite_ReadAdr, vel, 1);
    }
    velocity_LL = (uint16_t)vel[0];
}
//For V2 sensor

void LidarLite::refreshVelocityAdd(char currentAddres)
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};
    char read_dist[1]={GET_Distance2BReg};
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write((currentAddres<<1), write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write((currentAddres<<1), read_vel, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(((currentAddres<<1)|0x01), vel, 1);
    }
    velocity_LL = (uint16_t)vel[0];
}

void LidarLite::refreshRangeVelocity()
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};
    char read_dist[1]={GET_Distance2BReg};
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, read_dist, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(LIDARLite_ReadAdr, dist, 2);
    }
    distance_LL = ((uint16_t)dist[0] << 8) + (uint16_t)dist[1];
    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write(LIDARLite_WriteAdr, read_vel, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(LIDARLite_ReadAdr, vel, 1);
    }
    velocity_LL = (uint16_t)vel[0];
}

//For V2
void LidarLite::refreshRangeVelocityAdd(char currentAddres)
{
    uint8_t nackack;
    
    char write[2]={SET_CommandReg, AcqMode};
    char read_dist[1]={GET_Distance2BReg};
    char read_vel[1]={GET_VelocityReg};
    
    char dist[2];
    char vel[1];
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write((currentAddres<<1), write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write((currentAddres<<1), read_dist, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(((currentAddres<<1)|0x01), dist, 2);
    }
    distance_LL = ((uint16_t)dist[0] << 8) + (uint16_t)dist[1];
    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->write((currentAddres<<1), read_vel, 1);
    }    
    
    nackack=1;
    while(nackack !=0)
    {
        wait_ms(1);
        nackack = i2c_->read(((currentAddres<<1)|0x01), vel, 1);
    }
    velocity_LL = (uint16_t)vel[0];
}

void LidarLite::changeAddress(char currentAddres,char newAddress)
{   

    char read_vel[1]={0x96};
    char write[2]={0x1a,};
    char write_sno[2]={0x18,};
    char write_sno1[2]={0x19,};
    char write_D[2]={0x1e,0x08}; //disable Primary Address
    write[1]= newAddress;

    char s_no[2];
    uint8_t nackack=1;

    while(nackack !=0)
    {
            wait_ms(1);
            nackack = i2c_->read(((currentAddres<<1)|0x01), s_no, 2);
    }
    
    
    write_sno[1]=s_no[0];
    write_sno1[1]=s_no[0];
    
    nackack=1;
    while(nackack !=0)
    {
            wait_ms(1);
             i2c_->write((currentAddres<<1), write_sno, 2);//write 0x18
    }
    
    nackack=1;
    while(nackack !=0)
    {
            wait_ms(1);
             i2c_->write((currentAddres<<1), write_sno1, 2);//write 0x18
    }
    
    nackack=1;
    while(nackack !=0)
    {
            wait_ms(1);
             i2c_->write((currentAddres<<1), write, 2);
    }
    
    nackack=1;
    while(nackack !=0)
    {
            wait_ms(1);
            i2c_->write((currentAddres<<1), write, 2);
    }
    nackack=1;
    while(nackack !=0)
    {
            wait_ms(1);
            i2c_->write((currentAddres<<1), write_D, 2);
    }
    
}