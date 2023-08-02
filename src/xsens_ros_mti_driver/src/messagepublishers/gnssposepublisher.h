
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

#ifndef GNSSPOSEPUBLISHER_H
#define GNSSPOSEPUBLISHER_H

#include "packetcallback.h"
#include <geometry_msgs/PoseStamped.h>

struct GNSSPOSEPublisher : public PacketCallback
{
    ros::Publisher pub;
    std::string frame_id = DEFAULT_FRAME_ID;


    GNSSPOSEPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<geometry_msgs::PoseStamped>("gnss_pose", pub_queue_size);
        ros::param::get("~frame_id", frame_id);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsPositionLLA() && packet.containsOrientation())
        {
            geometry_msgs::PoseStamped msg;

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsVector p = packet.positionLLA();
            // publishing Lat/Long/Altitude as x,y z
            msg.pose.position.x = p[0];
            msg.pose.position.y = p[1];
            msg.pose.position.z = p[2];

            XsQuaternion q = packet.orientationQuaternion();
            // publishing Orientation as w,x,y,z
            msg.pose.orientation.w = q.w();
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();

            pub.publish(msg);
        }
    }
};

#endif
