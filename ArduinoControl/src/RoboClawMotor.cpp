#include "RoboClawMotor.h"

#define DZU 5

const float g_zero_threshold = 0.01;
const uint8_t g_zero_uint[] = {64-DZU,64+DZU};
const unsigned g_speed_max = 88000;

/////////////////////////////////
// channel

RoboClawMotor::RoboClawMotor(RoboClaw &roboclaw, unsigned char RoboClawAddress, uint8_t channel) 
    : m_roboclaw(roboclaw), m_channel(channel) {
    m_RoboClawAddress = RoboClawAddress;
}

void RoboClawMotor::speed(float s) {
    if (s > -g_zero_threshold && s < g_zero_threshold )
        s = 0;
    int32_t u = map(std::fabs(s), 0, 1.0, 0, g_speed_max);
    if (s < 0)
        u = -u;
    if (m_channel == 1)
        m_roboclaw.SpeedM1(m_RoboClawAddress, u);
    else if (m_channel == 2)
        m_roboclaw.SpeedM2(m_RoboClawAddress, u);
}

void RoboClawMotor::speed(uint8_t s) {
    

    if(s > g_zero_uint[0] && s<g_zero_uint[1]){
        s=64;//stop roboclaw
    }

    Serial.println("RCLAW: " + String(s));
    if (m_channel == 1)
        m_roboclaw.ForwardBackwardM1(m_RoboClawAddress, s);
    else if (m_channel == 2)
        m_roboclaw.ForwardBackwardM2(m_RoboClawAddress, s);
}