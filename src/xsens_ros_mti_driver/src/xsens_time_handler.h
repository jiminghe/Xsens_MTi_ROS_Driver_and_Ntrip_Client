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

private:
    std::string time_option;
    ros::Time firstUTCTimestamp;
    uint32_t prevSampleTimeFine;
    bool isFirstFrame;
    static const uint32_t ROLLOVER = 0xFFFFFFFF;  // 2^32-1
};

#endif // XSENS_TIME_HANDLER_H
