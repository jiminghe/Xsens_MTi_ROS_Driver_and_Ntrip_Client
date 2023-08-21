#include "xsens_time_handler.h"

XsensTimeHandler::XsensTimeHandler() : time_option(""), prevSampleTimeFine(0), isFirstFrame(true) {}

ros::Time XsensTimeHandler::convertUtcTimeToRosTime(const XsDataPacket &packet)
{
    std::lock_guard<std::mutex> lock(m_mutex);  // Lock the mutex at the beginning of the method
    if (time_option == "mti_utc" && packet.containsUtcTime())
    {
        //ROS_INFO("time_option is mti_utc");
        XsTimeInfo utcTime = packet.utcTime();
        struct tm timeinfo = {0};

        timeinfo.tm_year = utcTime.m_year - 1900;
        timeinfo.tm_mon = utcTime.m_month - 1;
        timeinfo.tm_mday = utcTime.m_day;
        timeinfo.tm_hour = utcTime.m_hour;
        timeinfo.tm_min = utcTime.m_minute;
        timeinfo.tm_sec = utcTime.m_second;

        time_t epochSeconds = timegm(&timeinfo);

        return ros::Time(epochSeconds, utcTime.m_nano);
    }
    else if (time_option == "mti_sampletime" && packet.containsSampleTimeFine())
    {
        uint32_t currentSampleTimeFine = packet.sampleTimeFine();

        if (isFirstFrame)
        {
            //ROS_INFO("time_option is mti_sampletime, first frame, timestamp: %u", currentSampleTimeFine);
            isFirstFrame = false;
            firstUTCTimestamp = ros::Time::now();
            prevSampleTimeFine = currentSampleTimeFine;
            return firstUTCTimestamp;
        }
        else
        {
            int64_t timeDiff = static_cast<int64_t>(currentSampleTimeFine) - static_cast<int64_t>(prevSampleTimeFine);

            // Checking for wraparound
            if (timeDiff < 0)
            {
                // Check if the difference is significant enough to be considered a wraparound
                if (timeDiff < -static_cast<int64_t>(m_RollOver / 2))
                {
                    // If wrap around occurred, adjust timeDiff
                    //ROS_INFO("time_option is mti_sampletime, Wraparound Detected. Current: %u, Previous: %u", currentSampleTimeFine, prevSampleTimeFine);
                    timeDiff += m_RollOver;
                }
                else
                {
                    ROS_WARN("Minor timestamp decrement detected but not considered as wraparound. Current: %u, Previous: %u", currentSampleTimeFine, prevSampleTimeFine);
                }
            }

            ros::Duration deltaTime(timeDiff * 0.0001); // Convert to seconds using the multiplier 0.0001
            firstUTCTimestamp += deltaTime;

            prevSampleTimeFine = currentSampleTimeFine;
            return firstUTCTimestamp;
        }
    }
    else
    {
        ROS_INFO("time_option is host controller time");
        return ros::Time::now(); // returns ros time
    }
}

void XsensTimeHandler::setTimeOption(const std::string &option)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    time_option = option;
}

void XsensTimeHandler::setRollover(const uint32_t &rollOver)
{
    std::lock_guard<std::mutex> lock(m_mutex); // Lock the mutex
    m_RollOver = rollOver;
}
