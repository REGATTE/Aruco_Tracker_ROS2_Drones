import rclpy
from rclpy.node import Node

from px4_msgs.msg import *

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.ARM_VEHICLE = 1.0
        self.DISARM_VEHICLE = 0.0
        self.now = 0
        self.arm_cnt = 0

        # PUBLISHERS
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, "fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "fmu/vehicle_command/in", 10)
        self.offboard_control_pub = self.create_publisher(OffboardControlMode, "fmu/offboard_control_mode/in", 10)

        # SUBSCRIBERS
        self.time_sub = self.create_subscription(Timesync, "fmu/timesync/out", self.timeCallback, 10)

    # def timeCallback function in subscriber
    def timeCallback(self, msg):
        if self.arm_cnt == 10: self.start_controller()
        if self.arm_cnt < 11: self.arm_cnt += 1
        self.now = msg.timestamp
        self.publish_offboard_control()
        self.fly()
        
    # def offboard control init
    def start_controller(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, self.ARM_VEHICLE) # set arm mode
        self.publish_offboard_control() # publish offboard messages
        
    def publish_offboard_control(self):
        offboard_ctrl_msg = OffboardControlMode()
        offboard_ctrl_msg.timestamp = self.now
        offboard_ctrl_msg.position = True
        offboard_ctrl_msg.velocity = False
        offboard_ctrl_msg.acceleration = False
        offboard_ctrl_msg.attitude = False
        offboard_ctrl_msg.body_rate = False
        self.offboard_control_pub.publish(offboard_ctrl_msg)         
        
    # function to publish PX4 commands to uORB messages
    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0):
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.now
        vehicle_command_msg.param1 = float(param1)
        vehicle_command_msg.param2 = float(param2)
        vehicle_command_msg.command = command
        vehicle_command_msg.target_system = 1
        vehicle_command_msg.target_component = 1
        vehicle_command_msg.source_system = 1
        vehicle_command_msg.source_component = 1
        vehicle_command_msg.from_external = True
        self.vehicle_command_pub.publish(vehicle_command_msg)

    # set waypoint (local)
    def fly(self):
        trajectory_msg = TrajectorySetpoint()
#        rates_msg = VehicleRatesSetpoint()

        trajectory_msg.timestamp = self.now
        # trajectory_msg.thrust = [0., 0., -.5]
        trajectory_msg.x = 2.0
        trajectory_msg.y = 2.0
        trajectory_msg.z = -3.0
        trajectory_msg.yaw = -3.14

#        rates_msg.timestamp = self.now
#        rates_msg.roll = 0.0
#        rates_msg.pitch = 0.0
#        rates_msg.yaw = 2.0

        self.trajectory_pub.publish(trajectory_msg)
#        self.rates_pub.publish(rates_msg)

# main function to run all       
def main(args=None):
    rclpy.init(args=args)
    offboard_control_node = OffboardControlNode()
    rclpy.spin(offboard_control_node)
    offboard_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
