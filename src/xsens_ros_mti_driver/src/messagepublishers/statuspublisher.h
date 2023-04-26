
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

#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "packetcallback.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <bitset>
using namespace std;

struct StatusPublisher : public PacketCallback
{
    ros::Publisher pub;


    StatusPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<diagnostic_msgs::KeyValue>("status", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsStatus())
        {
            diagnostic_msgs::KeyValue msg;

            msg.key = "StatusWord";
            // msg.value = bitset<32>(packet.status()).to_string();
            // pub.publish(msg);
            string statusMsg = "";
            uint32_t status = packet.status();

            bool gnssFix = status & (1<<2);
            if(gnssFix)
            {
                statusMsg += "GNSS: Fix, | ";
            }
            else
            {
                statusMsg += "GNSS: No, | ";
            }

            uint32_t rtkStatus = status & 0x18000000;
            switch (rtkStatus)
            {
                case 0:
                    statusMsg += "RTK: No, | ";
                    break;
                case 0x8000000:
                    statusMsg += "RTK: Floating, | ";
                    break;
                case 0x10000000:
                    statusMsg += "RTK: Fix, | ";
                    break;
            }

            msg.value = statusMsg;
            pub.publish(msg);


        }


    }
};

#endif
