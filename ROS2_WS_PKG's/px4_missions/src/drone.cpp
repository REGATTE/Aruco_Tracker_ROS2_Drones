#include "drone.hpp"

Drone::Drone(std::string vehicleName) : Node("drone")
{
	RCLCPP_INFO(this->get_logger(),  "Vehicle name: " + vehicleName);

	_offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>(vehicleName + "fmu/offboard_control_mode/in", 10);
	_trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>(vehicleName + "fmu/trajectory_setpoint/in", 10);
	_vehicle_command_publisher = this->create_publisher<VehicleCommand>(vehicleName + "fmu/vehicle_command/in", 10);

	// get common timestamp
	_timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("vhcl0/fmu/timesync/out", 10, [this](const px4_msgs::msg::Timesync::UniquePtr msg) {_timestamp.store(msg->timestamp);});

	_offboard_setpoint_counter = 0;

    //this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER, 175, 4);

	_timer = this->create_wall_timer(100ms, std::bind(&Drone::flight_mode_timer_callback, this));
}


void Drone::flight_mode_timer_callback()
{

	if (_offboard_setpoint_counter == 25)
	{
		this->setFlightMode(FlightMode::mOffboard);
	}

	if (_offboard_setpoint_counter == 50)
	{
		this->arm();
	}

	if (_offboard_setpoint_counter < 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0.0, 0.0, -5.0, 1.6);
	}

	if (_offboard_setpoint_counter < 300 && _offboard_setpoint_counter > 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(10, 0.0, -5.0, 0);
	}

	if (_offboard_setpoint_counter < 400 && _offboard_setpoint_counter > 300) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(10, 10, -5.0, 1.6);
	}

	if (_offboard_setpoint_counter < 500 && _offboard_setpoint_counter > 400) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0, 10, -5.0, 3.14);
	}

	if (_offboard_setpoint_counter < 600 && _offboard_setpoint_counter > 500) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0, 0, -5.0, -1.6);
	}

	if (_offboard_setpoint_counter == 600)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	_offboard_setpoint_counter++;
}


void Drone::setFlightMode(FlightMode mode)
{
	switch (mode)
	{
		case FlightMode::mOffboard:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			RCLCPP_INFO(this->get_logger(), "Offboard flight mode set");
			break;

		case FlightMode::mTakeOff:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 5.5, 1.5);
			RCLCPP_INFO(this->get_logger(), "TakeOff flight mode set");
			break;
			
		case FlightMode::mLand:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			RCLCPP_INFO(this->get_logger(), "Land flight mode set");
			break;
			
		case FlightMode::mReturnToLaunch:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
			RCLCPP_INFO(this->get_logger(), "Return to Launch flight mode set");
			break;
			
		default:
			RCLCPP_INFO(this->get_logger(), "No flight mode set");
	}

}


void Drone::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void Drone::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void Drone::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	_offboard_control_mode_publisher->publish(msg);
}


void Drone::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = x;
	msg.y = y;
	msg.z = z;
	msg.yaw = yaw; // [-PI:PI]

	_trajectory_setpoint_publisher->publish(msg);
}


void Drone::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5)
{
	VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
}