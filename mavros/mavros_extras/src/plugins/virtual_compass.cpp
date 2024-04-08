/**
 * @brief VirtualCompass plugin
 * @file virtual_compass.cpp
 * @author zhouzhiwen2000 <zhouzhiwen2000@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <geometry_msgs/Vector3Stamped.h>

namespace mavros {
namespace extra_plugins {

class VirtualCompassPlugin : public plugin::PluginBase {
public:
	VirtualCompassPlugin() : PluginBase(),
		sp_nh("~virtual_compass")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		virtual_compass_sub = sp_nh.subscribe("virtual_compass", 10, &VirtualCompassPlugin::vcompass_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle sp_nh;
	ros::Subscriber virtual_compass_sub;	//!< Subscriber to geometry_msgs/Point msgs

	void send_virtual_compass(Eigen::Vector3d  & v)
	{
		mavlink::common::msg::SCALED_IMU vc {};
		vc.xmag = v.x();
		vc.ymag = v.y();
		vc.zmag = v.z();
		UAS_FCU(m_uas)->send_message_ignore_drop(vc);
	}
	void vcompass_cb(const geometry_msgs::Point::ConstPtr & req)
	{
		Eigen::Vector3d tr;
		tf::pointMsgToEigen(*req, tr);

		send_virtual_compass(tr);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VirtualCompassPlugin, mavros::plugin::PluginBase)
