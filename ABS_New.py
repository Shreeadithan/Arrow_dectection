import rospy
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import math
import tf

class Detector:
    def __init__(self):
        rospy.init_node('depth_camera_detector', anonymous=True)
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/intel/color/image_raw", Image, self.image_callback)
        self.dep_sub = rospy.Subscriber("/intel/depth/image_raw", Image, self.depth_callback)
        self.pub = rospy.Publisher('galileo/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/galileo/laser_scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('galileo/imu', Imu, self.imu_callback)
        self.state = None
        self.twist = Twist()
        self.cv_img = None
        self.cv_depth = None
        self.position_initialized = False
        self.current_yaw = 0.0
        self.value = 1
        self.distance = float('inf')
        self.turning = False 
        self.cone_detected = False  
        self.run()

    def image_callback(self, data):
        if self.turning:
            return  

        self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)

       
        lower_black = np.array([0, 0, 0], dtype=np.uint8)
        upper_black = np.array([180, 255, 30], dtype=np.uint8)
        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        contours, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_contour = None
        image_height, image_width = self.cv_img.shape[:2]
        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            if len(approx) > 5:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(self.cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                detected_contour = contour

                if y + h > image_height * 0.8:  
                    self.cone_detected = True
                    break
                else:
                    self.cone_detected = False

                center_x = x + w // 2
                center_y = y + h // 2

        cv2.imshow("Detected Objects", self.cv_img)
        cv2.waitKey(1)
        self.arrow_contour = detected_contour
        if self.arrow_contour is not None and not self.cone_detected:
            x, y, w, h = cv2.boundingRect(self.arrow_contour)
            center_x = x + w // 2
            left_count = sum(1 for point in self.arrow_contour if point[0][0] < center_x)
            right_count = sum(1 for point in self.arrow_contour if point[0][0] > center_x)

            if left_count > right_count:
                self.state = "left"
            elif left_count < right_count:
                self.state = "right"

    def depth_callback(self, data):
        self.cv_depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def scan_callback(self, data):
        num_ranges = len(data.ranges)
        if num_ranges > 0:
            mid_index = num_ranges // 2
            start_index = mid_index - 150
            end_index = mid_index + 150
            relevant_ranges = data.ranges[start_index:end_index]
            self.distance = min(relevant_ranges)
            print(self.distance)
            if self.distance < 1.3:
                self.value = 0
            else:
                self.value = 2

    def odom_callback(self, data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        if not self.position_initialized:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.position_initialized = True

    def imu_callback(self, data):
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    
        

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cv_depth is not None and self.cv_img is not None:
                print("Main loop running")
                if self.cone_detected and self.distance<1.3:
                    print("Stop function is called")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                elif self.value == 0:
                    self.turning = True  
                    initial_yaw = self.current_yaw
                    target_yaw = initial_yaw + (math.pi / 2 if self.state == 'left' else -math.pi / 2)
                    while abs(self.current_yaw - target_yaw) > math.radians(1):
                        self.twist.linear.x = 0.1
                        self.twist.angular.z = 0.5 if self.state == 'left' else -0.5
                        self.pub.publish(self.twist)
                        rospy.sleep(0.1)

                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
                    self.turning = False  
                elif self.value == 2:
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
