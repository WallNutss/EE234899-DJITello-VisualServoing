import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

# Dirty way, is there the cleaner way? anyways, lot of pun way, as long is working, for now this should suffice
import sys
sys.path.append('/home/wallnuts/tello_ros_ws/src/tello_ros/tello_msgs')
from tello_msgs.srv import TelloAction

# load in the calibration data, its from calibration in 960x720
intrinsic_mat   = np.array([925.259979,         0.0, 491.398274,
                    0.0        , 927.502076, 371.463298,
                    0.0        , 0.0       , 1.0]).reshape(3,3)

distortion_coef = np.array([-0.018452, 0.108834, 0.003492, 0.001679, 0.0])

# So marker size is in meters?
MARKER_SIZE = 0.15 # centimeters (measure your printed marker size), Ideally have to try print them again, 15cm x 15cm should suffice
MARKER_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
PARAM_MARKERS = cv2.aruco.DetectorParameters()
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

        # For optical flow estimation
        self.oldCorner = np.array([[]])
        self.pointSelected = False
        self.arucoDetected = False
        self.stopCode      = False

        self.lk_params = dict(winSize=(20,20),
                              maxLevel=4,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10,0.01))
        self.old_gray_image = np.nan

        self.publisher = self.create_publisher(Float32MultiArray, '/corner_data', 10)
        self.publisherPosition = self.create_publisher(Float32MultiArray, '/position_data', 10)

        self.publisherPositionFlow = self.create_publisher(Float32MultiArray, '/position_data_flow', 10)



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
        # https://stackoverflow.com/questions/23853632/which-kind-of-interpolation-best-for-resizing-image
        # cv_image = cv2.resize(cv_image, (1280,720), interpolation=cv2.INTER_LINEAR)
        # finding the center of the frame
        (h,w) = cv_image.shape[:2]
        # Ploting the desired image location, center 640, 360
        n1 = ((w//2) - 50 , ((h//2) - 50) - 180) # [590, 310]
        n2 = ((w//2) - 50 , ((h//2) + 50) - 180) # [590, 410]
        n3 = ((w//2) + 50 , ((h//2) + 50) - 180) # [690, 410]
        n4 = ((w//2) + 50 , ((h//2) - 50) - 180) # [690, 310]

        #self.get_logger().info(f"\nn1: {n1}\nn2: {n2}\nn3: {n3}\nn4: {n4}\n")
        # EDITABLE
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

         # Trying to detect the marker with the object detector defined up there
        self.markerCorners, self.markerIds, rejectedCandidates = DETECTOR.detectMarkers(gray_image)
        self.stopCode = False

        # This connect to if there is detection
        if len(self.markerCorners) > 0:
            # Flattens the IDs
            # Getting the Pose of Estimation by getting the Translation and Rotation Vector
            # Based on the markerCorners, so Kalman Filter shoould estimate the points --> points_hat
            #rVec, tVec, trash = self.estimatePoseSingleMarkers(self.markerCorners, MARKER_SIZE, intrinsic_mat, distortion_coef)
            rVec, tVec,_ = cv2.aruco.estimatePoseSingleMarkers(self.markerCorners, MARKER_SIZE, intrinsic_mat, distortion_coef)
            try:
                mark = 2
                index = [value[0] for value in self.markerIds].index(mark)

                # Clockwise rotation in order start from top_left
                corners = self.markerCorners[index]
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # markerCorners is the list of corners of the detected markers. 
                # For each marker, its four corners are returned in their original order 
                # (which is clockwise starting with top left). So, the first corner is the top left corner, 
                # followed by the top right, bottom right and bottom left.
                top_left     = corners[0].ravel()
                top_right    = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left  = corners[3].ravel()

                # Euclidean Distance from aruco pose estimations (It's still estimation don't forgot!)
                # Z = round(math.sqrt(
                #     tVec[i][0][0] **2 + tVec[i][0][1] **2 + tVec[i][0][2] **2
                # ),5) # m (already in m)
                Z = round(tVec[index][0][2],5)


                floatArrayMsgData = Float32MultiArray()
                #data = self.flatten_nested_list([top_right, bottom_left, bottom_right, top_left])
                
                floatArrayMsgData.data = [float(top_left[0])             , float(top_left[1]), 
                                            float(bottom_left[0])          , float(bottom_left[1]), 
                                            float(bottom_right[0])         , float(bottom_right[1]), 
                                            float(top_right[0])            , float(top_right[1]),
                                            Z ]
                self.publisher.publish(floatArrayMsgData)

                # self.get_logger().info(str(distance))
                #point = cv2.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 7, 3)
                floatArrayMsgPosData = Float32MultiArray()
                floatArrayMsgPosData.data = [float(tVec[index][0][0]), float(tVec[index][0][1]), float(tVec[index][0][2])]
                self.publisherPosition.publish(floatArrayMsgPosData)

                floatArrayMsgPosDataFlow = Float32MultiArray()
                floatArrayMsgPosDataFlow.data = [float(tVec[index][0][0]), float(tVec[index][0][1]), float(tVec[index][0][2])]
                self.publisherPositionFlow.publish(floatArrayMsgPosDataFlow)

                cv2.putText(
                    cv_image,
                    f"X Relative: {round(tVec[index][0][0],5)} m",
                    (top_right[0]+20, top_right[1]+40),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (0,255,0),
                    2,
                    cv2.LINE_AA
                )
                cv2.putText(
                    cv_image,
                    f"Y Relative: {round(tVec[index][0][1],5)} m",
                    (top_right[0]+20, top_right[1]+60),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (0,255,0),
                    2,
                    cv2.LINE_AA
                )
                cv2.putText(
                    cv_image,
                    f"Z Relative: {round(tVec[index][0][2],5)} m",
                    (top_right[0]+20, top_right[1]+80),
                    cv2.FONT_HERSHEY_DUPLEX,
                    0.6,
                    (0,255,0),
                    2,
                    cv2.LINE_AA
                )

                # For Optical flow
                self.arucoDetected = True
                self.stopCode = True
                self.oldCorner = np.array([top_left, top_right, bottom_right, bottom_left],dtype=np.float32)

                corners = (np.array([[top_left, top_right, bottom_right, bottom_left]], dtype=np.float32),)

                # Draw the corners with convinient aruco library
                cv2.aruco.drawDetectedMarkers(cv_image, corners)

            except ValueError:
                pass
        if self.arucoDetected and self.stopCode == False:
            # Estimate marker corner if not available 
            new_corner, _, _ = cv2.calcOpticalFlowPyrLK(self.old_gray_image, gray_image, self.oldCorner, None, **self.lk_params)
            self.oldCorner  = new_corner
            new_corner = new_corner.astype(int)
            corners = (np.array([new_corner], dtype=np.float32),)

            # Estimate pose from marker corner estimation
            cv2.aruco.drawDetectedMarkers(cv_image, corners, borderColor=(255,0,0))
            rVec, tVec,_ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, intrinsic_mat, distortion_coef)

            # Clockwise rotation in order start from top_left
            corners = corners[0].reshape(4, 2)
            corners = corners.astype(int)

            # markerCorners is the list of corners of the detected markers. 
            # For each marker, its four corners are returned in their original order 
            # (which is clockwise starting with top left). So, the first corner is the top left corner, 
            # followed by the top right, bottom right and bottom left.
            top_left     = corners[0].ravel()
            top_right    = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left  = corners[3].ravel()

            # Euclidean Distance from aruco pose estimations (It's still estimation don't forgot!)
            Z = round(tVec[0][0][2],5)

            floatArrayMsgData = Float32MultiArray()
            
            floatArrayMsgData.data = [float(top_left[0])             , float(top_left[1]), 
                                        float(bottom_left[0])          , float(bottom_left[1]), 
                                        float(bottom_right[0])         , float(bottom_right[1]), 
                                        float(top_right[0])            , float(top_right[1]),
                                        Z ]
            self.publisher.publish(floatArrayMsgData)

            floatArrayMsgPosData = Float32MultiArray()
            floatArrayMsgPosData.data = [float(20.0), float(20.0), float(20.0)]
            self.publisherPosition.publish(floatArrayMsgPosData)

            floatArrayMsgPosDataFlow = Float32MultiArray()
            floatArrayMsgPosDataFlow.data = [float(tVec[0][0][0]), float(tVec[0][0][1]), float(tVec[0][0][2])]
            self.publisherPositionFlow.publish(floatArrayMsgPosDataFlow)

            cv2.putText(
                cv_image,
                f"X Relative: {round(tVec[0][0][0],5)} m",
                (top_right[0]+20, top_right[1]+40),
                cv2.FONT_HERSHEY_DUPLEX,
                0.6,
                (0,255,0),
                2,
                cv2.LINE_AA
            )
            cv2.putText(
                cv_image,
                f"Y Relative: {round(tVec[0][0][1],5)} m",
                (top_right[0]+20, top_right[1]+60),
                cv2.FONT_HERSHEY_DUPLEX,
                0.6,
                (0,255,0),
                2,
                cv2.LINE_AA
            )
            cv2.putText(
                cv_image,
                f"Z Relative: {round(tVec[0][0][2],5)} m",
                (top_right[0]+20, top_right[1]+80),
                cv2.FONT_HERSHEY_DUPLEX,
                0.6,
                (0,255,0),
                2,
                cv2.LINE_AA
            )

        #Draw the information
        # cv2.putText(
        #     cv_image,
        #     f"[ID]:[{ids[0]}]",
        #     top_right+20,
        #     cv2.FONT_HERSHEY_DUPLEX,
        #     0.6,
        #     (0,255,0),
        #     2,
        #     cv2.LINE_AA
        # )

        cv2.circle(cv_image, n1, 5, (255,0,0), 2)
        cv2.circle(cv_image, n2, 5, (255,0,0), 2)
        cv2.circle(cv_image, n3, 5, (255,0,0), 2)
        cv2.circle(cv_image, n4, 5, (255,0,0), 2)

        self.old_gray_image = gray_image.copy()
        key = cv2.waitKey(1)  # Refresh window
        # Safety Mechanism
        if key == ord("q"):
            raise SystemExit
        # Saving the display for logging
        if key == ord('s'):
            cv2.imwrite(f'./data/image{self.get_clock().now()}.png', cv_image)
            self.get_logger().info("Successfully saved the image!")
        # Resize the window frame to 60% Downscale for easy monitoring in the node
        cv_image = cv2.resize(cv_image, (576,360), interpolation=cv2.INTER_AREA) 
        # Display the image
        cv2.imshow('Image Display Node', cv_image)
            

    def call_tello_action_service(self,command):
        client = self.create_client(TelloAction, '/tello_action')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available, waiting again....')
        
        # Initiating Request
        request = TelloAction.Request()
        request.cmd = command

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
        aruco_detector.call_tello_action_service(command='rc 0 0 0 0')
        aruco_detector.call_tello_action_service(command='land')
        aruco_detector.call_tello_action_service(command='streamoff')
        rclpy.logging.get_logger("Leaving the process node").info('Done')
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
