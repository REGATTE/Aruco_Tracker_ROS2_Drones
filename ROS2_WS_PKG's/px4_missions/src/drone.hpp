#ifndef __DRONE_HPP__
#define __DRONE_HPP__

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Drone : public rclcpp::Node
{
public:
	Drone(std::string vehicleName = "");

private:

	// not yet used
	enum class FlightState
	{ sDisarmed, sArmed, sTakingOff, sReturningToLaunch, sLanding };

	// mTakeOff not yet working
	enum class FlightMode
	{ mOffboard, mTakeOff, mLand, mReturnToLaunch };

    void arm();
	void disarm();
    void setFlightMode(FlightMode mode);

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0);

	void flight_mode_timer_callback();

	rclcpp::TimerBase::SharedPtr _timer;
	uint64_t _offboard_setpoint_counter;   	// counter for the number of setpoints sent
	std::atomic<uint64_t> _timestamp;		// common synced timestamped

	rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;

};


#endif /*__DRONE_HPP__*/