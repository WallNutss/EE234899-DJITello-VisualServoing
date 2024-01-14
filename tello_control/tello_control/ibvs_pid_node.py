#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

def VelocityTwistMatrix(cRe,cTe):
    cMe = np.zeros((6,6))
    cMe[:3, :3] = cRe
    cMe[:3, 3:6] = np.matmul(cRe, cTe.reshape(-1,1))
    cMe[3:6, 3:6] = cRe
    return cMe

class IBVSPIDController(Node):
    def __init__(self,target):
        super().__init__('IBVS_Controller')
        self.get_logger().info("This is the start of IBVS PID Controller")
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/corner_data', self.vision_feedback, 10)
        self.publishererror = self.create_publisher(Float32MultiArray, '/error_data', 10)
        self.errData = Float32MultiArray()
        self.target = np.array(self.flatten_nested_list(target))
        #self.lamba = np.array([0.08, 0.095, 0.2, 0.04, 0.04, 0.04]).reshape(6,1)

        #self.focalLength = 0.025 #--> now its in m, aprox from dji tello specs # 904.91767127 # Its verified, its in pixel
        # new calibration data
        self.fx = 925.259979 # pixels
        self.fy = 927.502076 # pixels
        self.cx = 491.398274 # pixels
        self.cy = 371.463298 # pixels
        self.focalLength = 926 # Pixels
        #self.focalLength = (self.fx + self.fy)/2 # Pixels

        # {CF} --> {BF}
        cRe = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
        cTe = np.array([0,0,0])
        self.R = VelocityTwistMatrix(cRe,cTe)
        self.jacobian_end_effector = np.array([[1,0,0,0],
                                               [0,1,0,0],
                                               [0,0,1,0],
                                               [0,0,0,0],
                                               [0,0,0,0],
                                               [0,0,0,1]])
        
        self.last_time = self.get_clock().now().nanoseconds
        self.errorSum = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(8,1)
        self.errorPrev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(8,1)
    
    def image_jacobian_matrix(self, data):
        L = np.array([-self.focalLength/data[2], 0, data[0]/data[2], 
                      (data[0]*data[1])/self.focalLength, -(self.focalLength + data[0]**2/self.focalLength), data[1],
                      0, -self.focalLength/data[2], data[1]/data[2],
                      (self.focalLength + data[1]**2/self.focalLength), (-data[0]*data[1])/self.focalLength, -data[0]]).reshape(2,6)
        return L 

    def flatten_nested_list(self, nested_list):
        return [float(item) for sublist in nested_list for item in sublist]

    def vision_feedback(self, data):
        # self.get_logger().info("This is from IBVS Function\n")
        corner_data = np.array(data.data)

        # Compute control commands, Simple PID Control Trial
        current_time = self.get_clock().now().nanoseconds
        delta_time = (current_time - self.last_time) / 1e9 # Convert to seconds

        error_data = corner_data[0:self.target.shape[0]] - self.target
        error_data = error_data.reshape(-1,1)

        epsilon = np.sqrt(np.sum(error_data**2)/8)
        # Reason
        # With quadcopter as a non-linear system plus an image with also a non-linear system, it's getting harder
        # to control a quadcopter from an image. To cope with over overshoot, I divide it into two step flight
        # mechanism the cope with this, at least to reduce the burden of the calculation and easy implementation
        # as we get near the feature we want, at least in radius of 15px of each points, I decide to give the
        # all velocity reference 0. Now at least it seems to be working. 
        # Flight Mechanism 01
        if epsilon <= 20.0:
            cmd = np.array([0.0, 0.0, 0.0, 0.0])
            self.get_logger().info("01 Flight")
        # Flight Mechanism 02
        else:
            control_pid = (0.5*error_data + 0.002*(self.errorSum) + 0.05*(error_data-self.errorPrev)/delta_time)

            # Error data in form of shape 8x1 matrixs for Jacobian Pseudo Inverse Calculation
            jacobian_p1 = self.image_jacobian_matrix((corner_data[0],corner_data[1], corner_data[-1]))
            jacobian_p2 = self.image_jacobian_matrix((corner_data[2],corner_data[3], corner_data[-1]))
            jacobian_p3 = self.image_jacobian_matrix((corner_data[4],corner_data[5], corner_data[-1]))
            jacobian_p4 = self.image_jacobian_matrix((corner_data[6],corner_data[7], corner_data[-1]))
            
            Jacobian_ = np.vstack((jacobian_p1,jacobian_p2, jacobian_p3,jacobian_p4))
            Jacobian = np.linalg.pinv(np.matmul(np.matmul(Jacobian_, self.R),self.jacobian_end_effector))

            cmd = -0.15 * np.matmul(Jacobian, control_pid) # Camera Command U, maybe divide 0.2/100 will miracelyy goes to cm/s

            cmd = np.clip(cmd,-1.0,1.0)

        # Safety measure, try to cap them for testing and debugging,
        # Following the format given by tello_ros package, for cmd they map it to [-1,1]
        self.get_logger().info(f"Computational cmd:\n{cmd}\n")
        
        # Assign them to Twist 
        cmd_vel_msg = Twist()
        #self.get_logger().info(f"Shape cmd:\n{cmd.shape}\n")
        
        cmd_vel_msg.linear.x = float(cmd[0])
        cmd_vel_msg.linear.y = float(cmd[1])
        cmd_vel_msg.linear.z = float(cmd[2])
        cmd_vel_msg.angular.z = float(cmd[3]) #float(cmd[3])
        
        # Publish control commands
        self.publisher.publish(cmd_vel_msg)

        # Publish for logging purpose
        self.errData.data = self.flatten_nested_list(error_data)
        self.publishererror.publish(self.errData)

        # Update error
        self.errorSum = self.errorSum + error_data
        self.errorPrev = error_data
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)

    # Set the target position (replace with your desired coordinates)
    # This for a desired square location with square size of 50px each side,or offset in v-axis about 150px upward
    target_position2 = [[430,160], 
                       [430,260], 
                       [530,260], 
                       [530,160]] # Already corrected, its in pixel units
    # This is for offset in v-axis about 200px upward in 960x720 frame, spacing 100px
    target_position3 = [[430,110], 
                        [430,210], 
                        [530,210], 
                        [530,110]]
    # This is for offset in v-axis about 180px upward in 960x720 frame, spacing 100px
    target_position4 = [[430,130], 
                        [430,230], 
                        [530,230], 
                        [530,130]]
    
    ibvs_controller = IBVSPIDController(target_position3)
    rclpy.spin(ibvs_controller)
    ibvs_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

