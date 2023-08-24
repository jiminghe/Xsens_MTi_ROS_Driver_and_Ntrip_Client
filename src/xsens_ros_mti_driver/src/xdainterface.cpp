
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

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsxbusmessageid.h>
#include <xstypes/xsmessage.h>
#include <mavros_msgs/RTCM.h>

#include "messagepublishers/packetcallback.h"
#include "messagepublishers/accelerationpublisher.h"
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/freeaccelerationpublisher.h"
#include "messagepublishers/gnsspublisher.h"
#include "messagepublishers/imupublisher.h"
#include "messagepublishers/magneticfieldpublisher.h"
#include "messagepublishers/orientationincrementspublisher.h"
#include "messagepublishers/orientationpublisher.h"
#include "messagepublishers/pressurepublisher.h"
#include "messagepublishers/temperaturepublisher.h"
#include "messagepublishers/timereferencepublisher.h"
#include "messagepublishers/transformpublisher.h"
#include "messagepublishers/twistpublisher.h"
#include "messagepublishers/velocityincrementpublisher.h"
#include "messagepublishers/positionllapublisher.h"
#include "messagepublishers/velocitypublisher.h"
#include "messagepublishers/statuspublisher.h"
#include "messagepublishers/nmeapublisher.h"
#include "messagepublishers/gnssposepublisher.h"
#include "messagepublishers/utctimepublisher.h"
#include "xsens_log_handler.h"

#define XS_DEFAULT_BAUDRATE (115200)

XdaInterface::XdaInterface()
	: m_device(nullptr)
{
	ROS_INFO("Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	ROS_INFO("Cleaning up ...");
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);

	if (!rosPacket.second.empty())
	{
		for (auto &cb : m_callbacks)
		{
			cb->operator()(rosPacket.second, rosPacket.first);
		}
	}
}

void XdaInterface::registerPublishers(ros::NodeHandle &node)
{
	bool should_publish;

	if (ros::param::get("~pub_imu", should_publish) && should_publish)
	{
		registerCallback(new ImuPublisher(node));
	}
	if (ros::param::get("~pub_quaternion", should_publish) && should_publish)
	{
		registerCallback(new OrientationPublisher(node));
	}
	if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	{
		registerCallback(new AccelerationPublisher(node));
	}
	if (ros::param::get("~pub_angular_velocity", should_publish) && should_publish)
	{
		registerCallback(new AngularVelocityPublisher(node));
	}
	if (ros::param::get("~pub_mag", should_publish) && should_publish)
	{
		registerCallback(new MagneticFieldPublisher(node));
	}
	if (ros::param::get("~pub_dq", should_publish) && should_publish)
	{
		registerCallback(new OrientationIncrementsPublisher(node));
	}
	if (ros::param::get("~pub_dv", should_publish) && should_publish)
	{
		registerCallback(new VelocityIncrementPublisher(node));
	}
	if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	{
		registerCallback(new TimeReferencePublisher(node));
	}
	if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	{
		registerCallback(new TemperaturePublisher(node));
	}
	if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	{
		registerCallback(new PressurePublisher(node));
	}
	if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	{
		registerCallback(new GnssPublisher(node));
	}
	if (ros::param::get("~pub_twist", should_publish) && should_publish)
	{
		registerCallback(new TwistPublisher(node));
	}
	if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	{
		registerCallback(new FreeAccelerationPublisher(node));
	}
	if (ros::param::get("~pub_transform", should_publish) && should_publish)
	{
		registerCallback(new TransformPublisher(node));
	}
	if (ros::param::get("~pub_positionLLA", should_publish) && should_publish)
	{
		registerCallback(new PositionLLAPublisher(node));
	}
	if (ros::param::get("~pub_velocity", should_publish) && should_publish)
	{
		registerCallback(new VelocityPublisher(node));
	}
	if (ros::param::get("~pub_status", should_publish) && should_publish)
	{
		ROS_INFO("registerCallback StatusPublisher....");
		registerCallback(new StatusPublisher(node));
	}
	if (ros::param::get("~pub_nmea", should_publish) && should_publish)
	{
		ROS_INFO("registerCallback NMEAPublisher....");
		registerCallback(new NMEAPublisher(node));
	}
	if (ros::param::get("~pub_gnsspose", should_publish) && should_publish)
	{
		ROS_INFO("registerCallback GNSSPOSEPublisher....");
		registerCallback(new GNSSPOSEPublisher(node));
	}
	if (ros::param::get("~pub_utctime", should_publish) && should_publish)
	{
		ROS_INFO("registerCallback UTCTimePublisher....");
		registerCallback(new UTCTimePublisher(node));
	}
}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;
	XsBaudRate baudrate = XBR_Invalid;
	bool checkDeviceID = false;
	std::string deviceId = "";
	
	// Check if scanning is enabled
	bool scan_for_devices = false;
	ros::param::get("~scan_for_devices", scan_for_devices);

	if (!scan_for_devices)
	{
		// Read baudrate parameter if set
		if (ros::param::has("~baudrate"))
		{
			int baudrateParam = 0;
			ros::param::get("~baudrate", baudrateParam);
			ROS_INFO("Found baudrate parameter: %d", baudrateParam);
			baudrate = XsBaud::numericToRate(baudrateParam);
		}

		// Read device ID parameter
		if (ros::param::has("~device_id"))
		{
			ros::param::get("~device_id", deviceId);
			checkDeviceID = true;
			ROS_INFO("Found device ID parameter: %s.", deviceId.c_str());
		}

		// Read port parameter if set
		if (ros::param::has("~port"))
		{
			std::string portName;
			ros::param::get("~port", portName);
			ROS_INFO("Found port name parameter: %s", portName.c_str());
			mtPort = XsPortInfo(portName, baudrate);
			ROS_INFO("Scanning port %s ...", portName.c_str());
			if (!XsScanner::scanPort(mtPort, baudrate))
				return handleError("No MTi device found. Verify port and baudrate.");
			if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
				return handleError("No MTi device found with matching device ID.");
		}
	}
	else
	{
		ROS_INFO("Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");

	ROS_INFO("Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), XsBaud::rateToNumeric(mtPort.baudrate()));

	ROS_INFO("Opening port %s ...", mtPort.portName().toStdString().c_str());
	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	ROS_INFO("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	ROS_INFO("Measuring ..");
	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	bool should_log = false;
	if (ros::param::get("~should_log", should_log) && should_log)
	{
		XsensLogHandler logHandler;
		logHandler.prepareLogDirectory();
		std::string log_file = logHandler.getLogFileName();

		if (m_device->createLogFile(log_file) != XRV_OK)
			return handleError(std::string("Failed to create a log file! (%s)") + log_file);
		else
			ROS_INFO("Created a log file: %s", log_file.c_str());

		if (!m_device->startRecording())
			return handleError("Could not start recording");
	}

	return true;
}

bool XdaInterface::manualGyroBiasEstimation(uint16_t duration)
{
	XsMessage snd(XMID_SetNoRotation, sizeof(uint16_t));
	XsMessage rcv;
	snd.setDataShort(duration);
	if (!m_device->sendCustomMessage(snd, true, rcv, 1000))
		return false;

	ROS_INFO("Manual Gyro Bias Estimation sent.");
	return true;
}

void XdaInterface::callbackGyroBiasEstimation(const ros::TimerEvent &event)
{
	manualGyroBiasEstimation(1);
	ROS_INFO("MGBE callback triggered");
}

void XdaInterface::rtcmCallback(const mavros_msgs::RTCM::ConstPtr &msg)
{
	XsMessage rtcm(XMID_ForwardGnssData);
	uint16_t rtcmMessageLength = (const uint16_t)msg->data.size();
	rtcm.setDataBuffer((const uint8_t *)&msg->data[0], rtcmMessageLength, 0);
	XsMessage rcv;
	m_device->sendCustomMessage(rtcm, false, rcv, 0);
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	ROS_ERROR("%s", error.c_str());
	return false;
}
