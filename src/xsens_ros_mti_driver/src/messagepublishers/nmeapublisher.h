
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef NEMAPUBLISHER_H
#define NMEAPUBLISHER_H

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
    std::string frame_id = DEFAULT_FRAME_ID;
    ros::Time last_published_time;

    NMEAPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<nmea_msgs::Sentence>("nmea", pub_queue_size);
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        // Get the current time
        ros::Time current_time = ros::Time::now();

        // Calculate the time difference from the last published message
        double time_difference = (current_time - last_published_time).toSec();

        // If it has been at least 1 second since the last published message
        if (time_difference >= 1.0)
        {
            // Reset the last published time
            last_published_time = current_time;

            nmea_msgs::Sentence nmea_msg;

            nmea_msg.header.stamp = timestamp;
            nmea_msg.header.frame_id = frame_id;

            if (packet.containsRawGnssPvtData())
            {
                XsRawGnssPvtData gnssPvtData = packet.rawGnssPvtData();
                std::string gga_buffer;
                libntrip::generateGGA(gnssPvtData, &gga_buffer);

                nmea_msg.sentence = gga_buffer;

                pub.publish(nmea_msg);

                return;  // If this block is executed, immediately return to avoid executing the next block
            }
            else if (packet.containsUtcTime() && packet.containsLatitudeLongitude() && packet.containsStatus())
            {
                XsRawGnssPvtData gnssData;

                XsTimeInfo utcTime = packet.utcTime();
                
                // If the year is 1970, don't proceed with publishing.
                if(utcTime.m_year == 1970)
                    return;

                uint32_t status = packet.status();

                bool gnssFix = status & (1<<2);

                uint8_t fixType;

                if (gnssFix)
                {
                    fixType = 0x03; //3D-Fix
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
    }
};

#endif

