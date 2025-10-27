#ifndef __ROBOCLAW_channel_H__
#define __ROBOCLAW_channel_H__

#include <Arduino.h>
#include <cmath>


#include <RoboClaw.h>


/////////////////////////////////
// channel

class RoboClawMotor {
public:
    RoboClawMotor(RoboClaw &roboclaw, unsigned char RoboClawAddress, uint8_t channel); 
    void speed(float s); // -1.0 to +1.0
    void speed(uint8_t s);
private:
    RoboClaw &m_roboclaw;
    uint8_t m_channel;

    unsigned char m_RoboClawAddress;
};

#endif