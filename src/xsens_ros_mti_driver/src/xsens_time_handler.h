#ifndef XSENS_TIME_HANDLER_H
#define XSENS_TIME_HANDLER_H

#include <ros/ros.h>
#include <xstypes/xsdatapacket.h>

class XsensTimeHandler
{
public:
    XsensTimeHandler();
    ros::Time convertUtcTimeToRosTime(const XsDataPacket &packet);
    void setTimeOption(const std::string &option);
    void setRollover(const uint32_t &rollOver);

private:
    std::string time_option;
    ros::Time firstUTCTimestamp;
    uint32_t prevSampleTimeFine;
    bool isFirstFrame;
    uint32_t m_RollOver = 0xFFFFFFFF;  // 2^32-1
    mutable std::mutex m_mutex; // Mutex to protect the shared state
     
};

#endif // XSENS_TIME_HANDLER_H
