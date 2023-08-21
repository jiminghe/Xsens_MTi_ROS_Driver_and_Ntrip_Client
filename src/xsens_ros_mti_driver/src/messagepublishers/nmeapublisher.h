
//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef NEMAPUBLISHER_H
#define NEMAPUBLISHER_H

#include "packetcallback.h"
#include <nmea_msgs/Sentence.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "ntrip_util.h"

struct NMEAPublisher : public PacketCallback
{
    ros::Publisher pub;
    ros::Timer timer;
    ros::Time m_timeStamp;
    std::string frame_id = DEFAULT_FRAME_ID;

    XsDataPacket latest_packet; // to store the latest packet
    bool new_data_available = false;

    NMEAPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<nmea_msgs::Sentence>("nmea", pub_queue_size);
        ros::param::get("~frame_id", frame_id);

        // Set up a timer to trigger every 1 second
        timer = node.createTimer(ros::Duration(1.0), &NMEAPublisher::timerCallback, this);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        latest_packet = packet; // Update the latest packet
        m_timeStamp = timestamp;
        new_data_available = true;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        if (new_data_available)
        {
            processAndPublish(latest_packet);
            new_data_available = false;
        }
    }

    void processAndPublish(const XsDataPacket &packet)
    {
        nmea_msgs::Sentence nmea_msg;
        nmea_msg.header.stamp = m_timeStamp;
        nmea_msg.header.frame_id = frame_id;

        if (packet.containsRawGnssPvtData())
        {
            XsRawGnssPvtData gnssPvtData = packet.rawGnssPvtData();
            std::string gga_buffer;
            libntrip::generateGGA(gnssPvtData, &gga_buffer);

            nmea_msg.sentence = gga_buffer;
            pub.publish(nmea_msg);

            return; // If this block is executed, immediately return to avoid executing the next block
        }
        else if (packet.containsUtcTime() && packet.containsLatitudeLongitude() && packet.containsStatus())
        {
            XsRawGnssPvtData gnssData;

            XsTimeInfo utcTime = packet.utcTime();

             // If the year is 1970, don't proceed with publishing.
            if (utcTime.m_year == 1970)
                return;

            uint32_t status = packet.status();
            bool gnssFix = status & (1 << 2);

            uint8_t fixType;
            if (gnssFix)
            {
                fixType = 0x03; // 3D-Fix
            }
            else
            {
                fixType = 0x00;
            }

            XsVector latLon = packet.latitudeLongitude();
            int32_t longitude = static_cast<int32_t>(latLon[1] * 1e7);
            int32_t latitude = static_cast<int32_t>(latLon[0] * 1e7);

            gnssData.m_year = utcTime.m_year;
            gnssData.m_month = utcTime.m_month;
            gnssData.m_day = utcTime.m_day;
            gnssData.m_hour = utcTime.m_hour;
            gnssData.m_min = utcTime.m_minute;
            gnssData.m_sec = utcTime.m_second;
            gnssData.m_nano = utcTime.m_nano;
            gnssData.m_valid = utcTime.m_valid;

            gnssData.m_fixType = fixType;
            gnssData.m_lon = longitude;
            gnssData.m_lat = latitude;

            std::string gga_buffer;
            libntrip::generateGGA(gnssData, &gga_buffer);

            nmea_msg.sentence = gga_buffer;
            pub.publish(nmea_msg);
        }
    }
};

#endif

