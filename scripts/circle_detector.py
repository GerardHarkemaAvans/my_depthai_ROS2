#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from sensor_msgs.msg import PointCloud2
import numpy as np

import ros2_numpy

import cv2
from cv_bridge import CvBridge

import tf2_ros
import geometry_msgs.msg

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CircleDetector(Node):

    def __init__(self):
        super().__init__('my_subscriber')

        self.bridge = CvBridge()
        self.image = None
        self.detected_circles = None
        self.display_image = False

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        if 0:
            self.subPointcloud = self.create_subscription(
                PointCloud2,
                '/stereo/points',
                self.pointcloud_callback,
                10)
            self.subPointcloud  # prevent unused variable warning		
        else:
            self.subPointcloud = self.create_subscription(
                PointCloud2,
                '/stereo/points',
                callback = self.pointcloud_callback,
                qos_profile=qos_profile
            )



        self.subImage = self.create_subscription(
            Image,
            'color/image_rect',
            self.image_callback,
            10)
        self.subImage  # prevent unused variable warning

        self.pubDetectedCircleImage = self.create_publisher(Image, 'color/detected_circles', 10)
    pass
    
    def pointcloud_callback(self, point_cloud_msg):
        #rospy.loginfo("PCL Callback")

        #pc_list = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud_msg, remove_nans = False )
        pc_list = ros2_numpy.point_cloud2.point_cloud2_to_array(point_cloud_msg)

        #print(pc_list)

        if self.detected_circles is not None:


            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(self.detected_circles))

            i = 1
            for pt in detected_circles[0, :]:
                y, x, r = pt[0], pt[1], pt[2] # This is magic, x & y zijn omgedraaid!

                x1 = x - r
                y1 = y - r
                x2 = x + r
                y2 = y + r
                if((x1 >= point_cloud_msg.width) or (x2 >= point_cloud_msg.width)):
                    continue
                if((y1 >= point_cloud_msg.height) or (y2 >= point_cloud_msg.height)):
                    continue

                min_x, min_y, min_z = 0,0,9000
                for it_x in range(x1, x2):
                    for it_y in range(y1, y2):
                        #curr_pos = pc_list[it_y][it_x] # hoe zit dit?
                        
                        xyz_data = pc_list["xyz"]
                        print(len(xyz_data))
                        print(xyz_data)
                        curr_pos = xyz_data[it_y][it_x] # hoe zit dit?
                        if curr_pos[0] is not None:
                            if(curr_pos[2] < min_z):
                                min_x = curr_pos[0]
                                min_y = curr_pos[1]
                                min_z = curr_pos[2]

                curr_pos = min_x, min_y, min_z
                #rospy.loginfo(curr_pos)

                t = TransformStamped()

                child_frame_id = '%s%i' % ("circle_", i)
                i=i+1

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'oak_model_origin' # nog even de juiste selecteren!!!!!
                t.child_frame_id = child_frame_id
                t.transform.translation.x = min_x
                t.transform.translation.y = min_y
                t.transform.translation.z = min_z
                q = t.transformations.quaternion_from_euler(0, 0, 0) 
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.tf_broadcaster.sendTransform(t)
                print(".")

        pass

    def image_callback(self, image_msg):

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        # Convert to grayscale.
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blur using 11 * 11 kernel.
        gray_blurred = cv2.GaussianBlur(gray,(11,11),cv2.BORDER_DEFAULT)

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
        cv2.HOUGH_GRADIENT, 1, 100, param1 = 100,
        param2 = 30, minRadius = 50, maxRadius = 400)

        #cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );


        # Draw circles that are detected.
        if detected_circles is not None:

            # Convert the circle parameters a, b and r to integers.
            self.detected_circles = np.uint16(np.around(detected_circles))

            for pt in self.detected_circles[0, :]:
                x, y, r = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)

                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(cv_image, (x, y), 5, (0, 255, 0), 8)

        if self.display_image:
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        image_message= self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

        self.pubDetectedCircleImage.publish(image_message)
        pass

def main(args=None):
    rclpy.init(args=args)

    circle_detector = CircleDetector()

    rclpy.spin(circle_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
