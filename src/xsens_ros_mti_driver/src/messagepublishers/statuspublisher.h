
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
