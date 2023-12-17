#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math
import numpy as np
from scipy.linalg import pinvh


class IBVSPIDController(Node):
    def __init__(self,target):
        super().__init__('IBVS_Controller')
        self.get_logger().info("This is the start of IBVS PID Controller")
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/corner_data', self.vision_feedback, 10)
        self.publishererror = self.create_publisher(Float32MultiArray, '/error_data', 10)
        self.errData = Float32MultiArray()
        self.target = np.array(self.flatten_nested_list(target))
        # 0.2 its ok, but the more you put >0.2 the more it breaks, better put them around 0.01 ~ 0.15
                            # y[0]   z[1] x[2]  wy wz  wx
        self.lamba = np.array([0.08, 0.095, 0.2, 0, 1.2, 2]).reshape(6,1)

        #self.focalLength = 0.025 #--> now its in m, aprox from dji tello specs # 904.91767127 # Its verified, its in pixel
        self.focalLength = 904.91767127 # Pixels
        self.fx = 904.91767127 # pixels
        self.fy = 904.09119851 # pixels
        self.cx = 503.21829022 # pixels
        self.cy = 355.3507261 # pixels

        # {CF} --> {BF}
        self.R = np.array([ 0, 0, 1, 0, 0, 0,
                           -1, 0, 0, 0, 0, 0,
                            0,-1, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 1,
                            0, 0, 0,-1, 0, 0,
                            0, 0, 0, 0,-1, 0,]).reshape(6,6)
        
        self.last_time = self.get_clock().now().nanoseconds
        self.errorSum = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(8,1)
        self.errorPrev = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(8,1)
    
    def image_jacobian_matrix(self, data):
        L = np.array([-self.focalLength/data[2], 0, data[0]/data[2], 
                      (data[0]*data[1])/self.focalLength, -(self.focalLength + data[0]**2/self.focalLength), data[1],
                      0, -self.focalLength/data[2], data[1]/data[2],
                      (self.focalLength + data[1]**2/self.focalLength), (-data[0]*data[1])/self.focalLength, -data[0]]).reshape(2,6)
        return L 

    def image_jacobian_matrix2(self, data):
        L = np.array([1/data[2], 0 , data[0]/data[2],
                      data[0]*data[1], -(1+data[0]**2), data[1],
                      0, -1/data[2], data[1]/data[2],
                      (1+data[1]**2), -data[0]*data[1], -data[0]]).reshape(2,6)
        return L    

    def image_projection(self, principal, focallength, pixel):
        return (pixel - principal)/focallength

    def flatten_nested_list(self, nested_list):
        return [float(item) for sublist in nested_list for item in sublist]
    
    def saturationCommand(self, U):
        return np.clip(U,-0.2,0.2)

    
    def vision_feedback(self, data):
        # self.get_logger().info("This is from IBVS Function\n")
        corner_data = np.array(data.data)

        # Compute control commands, Simple PID Control Trial
        current_time = self.get_clock().now().nanoseconds
        delta_time = (current_time - self.last_time) / 1e9 # Convert to seconds

        error_data = corner_data[0:self.target.shape[0]] - self.target
        error_data = error_data.reshape(-1,1)

        # control = -1*(0.005*error_data + 0.0002*(error_data - self.errorPrev)/delta_time)
        control_pid = (0.45*error_data + 0.002*(self.errorSum) + 0.05*(error_data-self.errorPrev)/delta_time)

        '''Normalize first
        # corner_data[0:7] = np.array([self.image_projection(self.cx, self.fx, corner_data[0]),
        #                              self.image_projection(self.cy, self.fy, corner_data[1]),
        #                              self.image_projection(self.cx, self.fx, corner_data[2]),
        #                              self.image_projection(self.cy, self.fy, corner_data[3]),
        #                              self.image_projection(self.cx, self.fx, corner_data[4]),
        #                              self.image_projection(self.cy, self.fy, corner_data[5]),
        #                              self.image_projection(self.cx, self.fx, corner_data[6]),
        #                              self.image_projection(self.cy, self.fy, corner_data[7])])
        '''

        # Error data in form of shape 8x1 matrixs for Jacobian Pseudo Inverse Calculation
        jacobian_p1 = self.image_jacobian_matrix((corner_data[0],corner_data[1], corner_data[-1]))
        jacobian_p2 = self.image_jacobian_matrix((corner_data[2],corner_data[3], corner_data[-1]))
        jacobian_p3 = self.image_jacobian_matrix((corner_data[4],corner_data[5], corner_data[-1]))
        jacobian_p4 = self.image_jacobian_matrix((corner_data[6],corner_data[7], corner_data[-1]))
        
        Jacobian_ = np.vstack((jacobian_p1,jacobian_p2, jacobian_p3,jacobian_p4))
        Jacobian = np.linalg.pinv(Jacobian_)
        #Jacobian = np.matmul(np.linalg.pinv(np.matmul(Jacobian_.T, Jacobian_)),Jacobian_.T)
        #self.get_logger().info(f"Jacobian Matrix : {Jacobian_}\n")
        # #self.get_logger().info(f"Moore-Penrose Pseudo Jacobian : {Jacobian}\n")
        # self.get_logger().info(f"Error data: {error_data}\n")


        # np.set_printoptions(suppress=True)
        #cmd = -self.lamba * np.matmul(Jacobian, error_data) # Camera Command U
        cmd = -self.lamba * np.matmul(Jacobian, control_pid) # Camera Command U
        #cmd = np.matmul(Jacobian, control)
        #cmd = -self.lamba * (Jacobian @ error_data)
        #self.get_logger().info(f"Computational cmd: {cmd}\n")

        # Compute simple control commands (proportional control), transfer to body frame
        cmd = np.matmul(self.R, cmd)

        # Safety measure, try to cap them for testing and debugging,
        # Following the format given by tello_ros package, for cmd they map it to [-1,1]
        cmd = self.saturationCommand(cmd)
        
        # Assign them to Twist 
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x  = round(float(cmd[0]),3)
        cmd_vel_msg.linear.y  = round(float(cmd[1]),3)
        cmd_vel_msg.linear.z  = round(float(cmd[2]),3)
        cmd_vel_msg.angular.z = round(float(cmd[5]),3)
        
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
    target_position = [[490,210], 
                       [490,510], 
                       [790,510], 
                       [790,210]] # Already corrected, it in pixel units
    
    # # Try Projection Transformation at Target
    # target_position = [[490,210], 
    #                    [490,510], 
    #                    [632,408], 
    #                    [632,168]]
 
    ibvs_controller = IBVSPIDController(target_position)
    rclpy.spin(ibvs_controller)
    ibvs_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

