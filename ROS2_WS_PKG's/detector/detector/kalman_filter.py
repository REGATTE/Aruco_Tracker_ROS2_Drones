# ROS 2
import rclpy
from rclpy.node import Node

# Messages
from custom_messages.msg import States

# Python
import numpy as np

# defining PI as a constant
PI = 3.14159265358979323846

class Kalman_Filter(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        print("init")

        # subscriptions
        self.subscription_states = self.create_subscription(States, "/states", self.sub_states_callback, 10)
        self.subscription_states

        # publishers
        self.publish_states = self.create_publisher(States, "/predicted_param", 10)

        self.delta_time = 0.1
        self.first_iteraction = 0

        # Posterior1 estimate covariance matrix initialization
        self.T = np.matrix([[2,0,0,0,0,0,0,0,0,0],     # Xc
                        [0,2,0,0,0,0,0,0,0,0],     # Yc
                        [0,0,5,0,0,0,0,0,0,0],     # W
                        [0,0,0,5,0,0,0,0,0,0],     # H
                        [0,0,0,0,5.625,0,0,0,0,0], # theta
                        [0,0,0,0,0,1e-3,0,0,0,0],  # Xc'
                        [0,0,0,0,0,0,1e-3,0,0,0],  # Yc'
                        [0,0,0,0,0,0,0,1e-3,0,0],  # W'
                        [0,0,0,0,0,0,0,0,1e-3,0],  # H'
                        [0,0,0,0,0,0,0,0,0,1e-3]])  # theta'
        print("======================================")
        print("T")
        print(self.T)

        # covariance of the process noise initialization
        self.Q = 1e-4 * np.identity(10)
        print("======================================")
        print("Q")
        print(self.Q)

        # covariance of the observation noise initialization
        self.R = 1e-2 * np.identity(5)
        self.R[0, 0] = 1e-4
        self.R[1, 1] = 1e-4
        print("======================================")
        print("R")
        print(self.R)

        # State vector initialization
        self.X = np.zeros((10, 1))
        self.X = np.mat(self.X)
        print("======================================")
        print("X")
        print(self.X)

        # Innovation vectot initialization
        self.Z = np.zeros((5,1))
        self.Z = np.mat(self.Z)
        print("======================================")
        print("Z")
        print(self.Z)

        # Covariance of the innovation initialization
        self.S1 = np.zeros((5,5))
        self.S1 = np.mat(self.S1)
        print("======================================")
        print("S1")
        print(self.S1)

        # kalman gain initialization
        self.Kg = np.zeros((5, 5))
        self.Kg = np.mat(self.Kg)
        print("======================================")
        print("Kg")
        print(self.Kg)

        # State transition matrix initialization
        self.A = np.mat([[1,0,0,0,0,self.delta_time,0,0,0,0],  # Xc
                        [0,1,0,0,0,0,self.delta_time,0,0,0],    # Yc
                        [0,0,1,0,0,0,0,self.delta_time,0,0],    # W
                        [0,0,0,1,0,0,0,0,self.delta_time,0],    # H
                        [0,0,0,0,1,0,0,0,0,self.delta_time],    # theta
                        [0,0,0,0,0,1,0,0,0,0],                  # Xc'
                        [0,0,0,0,0,0,1,0,0,0],                  # Yc'
                        [0,0,0,0,0,0,0,1,0,0],                  # W'
                        [0,0,0,0,0,0,0,0,1,0],                  # H'
                        [0,0,0,0,0,0,0,0,0,1]])                  # theta'
        print("======================================")
        print("A")
        print(self.A)

        # Observation model initialization
        self.H =  np.mat([[1,0,0,0,0,0,0,0,0,0],   # Xc
                            [0,1,0,0,0,0,0,0,0,0],  # Yc
                            [0,0,1,0,0,0,0,0,0,0],  # W
                            [0,0,0,1,0,0,0,0,0,0],  # H
                            [0,0,0,0,1,0,0,0,0,0]])  # theta
        print("======================================")
        print("H")
        print(self.H)


    def sub_states_callback(self, msg):
        # print(msg.xc)
        prediction = States()
        measurement = np.zeros((5,1))

        # If it is the first iteration, 
        # initialize the states with the first observation
        if(self.first_iteraction) == 0:
            self.X[0,0] = msg.xc
            self.X[1,0] = msg.yc
            self.X[2,0] = msg.w
            self.X[3,0] = msg.h
            self.X[4,0] = msg.theta
            self.X[5,0] = 0
            self.X[6,0] = 0
            self.X[7,0] = 0
            self.X[8,0] = 0
            self.X[9,0] = 0
            self.first_iteraction = 1
        
        # Prediction Step
        self.X = self.A * self.X
        print("======================================")
        print("X Prediction")
        print(self.X)

        self.T = (((self.A * self.T)*self.A.transpose()) + self.Q)
        print("======================================")
        print("T Prediction")
        print(self.T)

        # Update step, 
        # assign the observations to the measurement vector 
        measurement[0,0] = msg.xc
        measurement[1,0] = msg.yc
        measurement[2,0] = msg.w
        measurement[3,0] = msg.h
        measurement[4,0] = msg.theta
        print("======================================")
        print("Updated")
        print(measurement)

        if((measurement[0]>0)&(measurement[1]>0)&(measurement[2]>0)&(measurement[3]>0)):
            self.Z = (measurement - (self.H * self.X))
            self.S1 = (((self.H * self.T) * self.H.transpose()) + self.R)
            self.Kg = ((self.T * self.H.transpose()) * np.linalg.inv(self.S1))
            self.X = (self.X + (self.Kg * self.Z))
            self.T = ((np.eye(10) - (self.Kg * self.H)) * self.T)
        
        prediction.xc = int(self.X[0,0])
        print(prediction.xc)
        prediction.yc = int(self.X[1,0])
        print(prediction.yc)
        prediction.w = int(self.X[2,0])
        print(prediction.w)
        prediction.h = int(self.X[3,0])
        print(prediction.h)
        prediction.theta = int(self.X[4,0])
        print(prediction.theta)

        self.publish_states.publish(prediction)

def main(args = None):
    rclpy.init(args = args)
    kalman_filter = Kalman_Filter()
    rclpy.spin(kalman_filter)

    kalman_filter.destroy_node()
    rclpy.shutdown

if __name__ == "__main__":
    main()