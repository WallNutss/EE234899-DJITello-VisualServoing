import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv2 import aruco
import numpy as np
from std_msgs.msg import Float32MultiArray, Empty
import math

# Dirty way, is there the cleaner way? anyways, lot of pun way, as long is working, for now this should suffice
import sys
sys.path.append('/home/wallnuts/tello_ros_ws/src/tello_ros/tello_msgs')
from tello_msgs.srv import TelloAction

# load in the calibration data
calib_data_path = "/home/wallnuts/tello_ros_ws/src/tello_vision/data_calibration_camera/MultiMatrix.npz"
calib_data = np.load(calib_data_path)

cam_mat,dist_coef,r_vectors, t_vectors = calib_data["camMatrix"], calib_data["distCoef"],calib_data["rVector"],calib_data["tVector"]

MARKER_SIZE = 15 # centimeters (measure your printed marker size), Ideally have to try print them again, 15cm x 15cm should suffice


MARKER_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
PARAM_MARKERS = aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(MARKER_DICT, PARAM_MARKERS)


class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.get_logger().info("Press 'q' to exit the node!")
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Replace with your actual image topic
            self.image_callback,
            10
        )
        #data
        self.markerCorners = np.nan
        self.alpha = 0.8
        self.kf = cv2.KalmanFilter(4,2) # There is 2 measurements, which is 1 points consist 2 coordinates system points
        self.kf.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kf.transitionMatrix  = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)

        self.publisher = self.create_publisher(Float32MultiArray, '/corner_data', 10)
        self.dataCorner = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.cv_bridge = CvBridge()
        self.currentTime = self.get_clock().now()

    def predict(self, coord):
        measured = np.array([[np.float32(coord[0])], [np.float32(coord[1])]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x,y = int(predicted[0]), int(predicted[1])

        return x,y

        # https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    def estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def flatten_nested_list(self, nested_list):
        return [float(item) for sublist in nested_list for item in sublist]

    def normalized(self, data):
        return [data[0]/1280, data[1]/720]
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # resize them to 1280x720 original datasheet, becasue the driver put them on 960x720
        cv_image = cv2.resize(cv_image, (1280,720), interpolation=cv2.INTER_AREA)
        # finding the center of the frame
        (h,w) = cv_image.shape[:2]
        # Ploting the desired image location
        n1 = ((w//2) - 50 , (h//2) - 50) # [490, 210]
        n2 = ((w//2) - 50 , (h//2) + 50) # [490, 510]
        n3 = ((w//2) + 50 , (h//2) + 50) # [790, 510]
        n4 = ((w//2) + 50 , (h//2) - 50) # [790, 210]

        #self.get_logger().info(f'n1 : {n1}\nn2: {n2}\nn3 :{n3}\nn4 : {n4}')

        cv2.circle(cv_image, n1, 5, (255,0,0), 2)
        cv2.circle(cv_image, n2, 5, (255,0,0), 2)
        cv2.circle(cv_image, n3, 5, (255,0,0), 2)
        cv2.circle(cv_image, n4, 5, (255,0,0), 2)
        # Ploting the desired image location
        # cv2.putText(cv_image, f"n1", n1, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"n2", n2, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"n3", n3, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)
        # cv2.putText(cv_image, f"n4", n4, cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,255,0),1, cv2.LINE_AA)

        # EDITABLE
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
         # Trying to detect the marker with the object detector defined up there
        self.markerCorners, self.markerIds, rejectedCandidates = DETECTOR.detectMarkers(gray_image)

        # This connect to if there is detection
        if len(self.markerCorners) > 0:
            # Flattens the IDs
            # Getting the Pose of Estimation by getting the Translation and Rotation Vector
            # Based on the markerCorners, so Kalman Filter shoould estimate the points --> points_hat
            rVec, tVec, trash = self.estimatePoseSingleMarkers(self.markerCorners, MARKER_SIZE, cam_mat, dist_coef)
            # Loop over the IDs
            total_markers = range(0, self.markerIds.size)
            for ids, corners, i in zip(self.markerIds, self.markerCorners, total_markers):
                # Draw the corners with convinient aruco library
                aruco.drawDetectedMarkers(cv_image, self.markerCorners)

                # Clockwise rotation in order start from top_left
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
 
                top_left     = corners[0].ravel()
                top_right    = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left  = corners[3].ravel()

                # top_left_predicted = self.predict(top_left)
                # top_right_predicted = self.predict(top_right)
                # bottom_right_predicted = self.predict(bottom_right)
                # bottom_left_predicted = self .predict(bottom_left)

                # Euclidean Distance from aruco pose estimations (It's still estimation don't forgot!)
                Z = round(math.sqrt(
                    tVec[i][0][0] **2 + tVec[i][1][0] **2 + tVec[i][2][0] **2
                )/100,3) # cm --> m (for now its convert to m)


                floatArrayMsgData = Float32MultiArray()
                #data = self.flatten_nested_list([top_right, bottom_left, bottom_right, top_left])
                
                floatArrayMsgData.data = [float(top_left[0]), float(top_left[1]), 
                                          float(bottom_left[0]), float(bottom_left[1]), 
                                          float(bottom_right[0]), float(bottom_right[1]), 
                                          float(top_right[0]), float(top_right[1]), Z]
                
                self.publisher.publish(floatArrayMsgData)

                # self.get_logger().info(str(distance))
                point = cv2.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 7, 3)


                #Draw number for debug
                # cv2.putText(cv_image, f"top_left", top_left_predicted, cv2.FONT_HERSHEY_DUPLEX, 0.6, (61,7,219),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_left", bottom_left_predicted, cv2.FONT_HERSHEY_DUPLEX, 0.6, (13,105,134),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_right", bottom_right_predicted, cv2.FONT_HERSHEY_DUPLEX, 0.6, (210,199,142),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"top_right", top_right_predicted, cv2.FONT_HERSHEY_DUPLEX, 0.6, (7,165,219),1, cv2.LINE_AA)

                #Draw number for debug
                # cv2.putText(cv_image, f"top_left", top_left, cv2.FONT_HERSHEY_DUPLEX, 0.6, (61,7,219),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_left", bottom_left, cv2.FONT_HERSHEY_DUPLEX, 0.6, (13,105,134),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"bottom_right", bottom_right, cv2.FONT_HERSHEY_DUPLEX, 0.6, (210,199,142),1, cv2.LINE_AA)
                # cv2.putText(cv_image, f"top_right", top_right, cv2.FONT_HERSHEY_DUPLEX, 0.6, (7,165,219),1, cv2.LINE_AA)

                # Draw the information
                cv2.putText(
                    cv_image,
                    f"[ID]:[{ids[0]}] Distance: {Z} m",
                    top_right+20,
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (0,255,0),
                    2,
                    cv2.LINE_AA
                )
        else:
            # Low filter
            pass


        # Resize the window frame to 60% Downscale for easy monitoring in the node
        cv_image = cv2.resize(cv_image, (640,360), interpolation=cv2.INTER_AREA) 
        # Display the image
        cv2.imshow('Image Display Node', cv_image)
        key = cv2.waitKey(1)  # Refresh window
        if key == ord("q"):
            raise SystemExit
        # Saving the display for logging
        if key == ord('s'):
            cv2.imwrite(f'./data/image{self.currentTime}.png', cv_image)
            self.get_logger().info("Successfully saved the image!")
            

    def call_tello_action_service(self,cmd):
        client = self.create_client(TelloAction, '/tello_action')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available, waiting again....')
        
        # Initiating Request
        request = TelloAction.Request()
        request.cmd = cmd

        # Sending the request from client to origin
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self,future)

        # Check status
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().error('Service call failed')



def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ImageDisplayNode()
    try:
        rclpy.spin(aruco_detector)
    except SystemExit:                 # <--- process the exception 
        # For safety, land the drone
        aruco_detector.call_tello_action_service(cmd='land')
        aruco_detector.call_tello_action_service(cmd='streamoff')
        rclpy.logging.get_logger("Leaving the process node").info('Done')
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
