#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray
from sensor_msgs.msg import Image

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

import cv2
from cv_bridge import CvBridge

import json

class PublischBoundingBoxes(Node):

    def __init__(self):
        super().__init__('my_subscriber')

        self.bridge = CvBridge()
        self.image = None

        self.declare_parameter("resourceBaseFolder", "");
        path = self.get_parameter("resourceBaseFolder").get_parameter_value().string_value
        print(path)
        self.declare_parameter("nnConfig", "");
        nnConfig = self.get_parameter("nnConfig").get_parameter_value().string_value
        print(nnConfig)
        nnConfigPath = path + '/' + nnConfig
        print(nnConfigPath)
        # Opening JSON file
        f = open(nnConfigPath)
        
        # returns JSON object as 
        # a dictionary
        data = json.load(f)
       
        # Closing file
        f.close()

        #print(data)
        mappings = data['mappings']
        self.labels = mappings['labels']
        #print(labels)

        #self.print(labels[0])

        self.subSpatialDetection = self.create_subscription(
            SpatialDetectionArray,
            'color/yolov4_Spatial_detections',
            self.spatial_dections_callback,
            10)
        self.subSpatialDetection  # prevent unused variable warning

        self.subImage = self.create_subscription(
            Image,
            'color/image_rect',
            self.image_callback,
            10)
        self.subSpatialDetection  # prevent unused variable warning

        self.pubImageBoudingBoxes = self.create_publisher(Image, 'color/image_w_bouding_boxes', 10)
            #    self.pubTextMarkers = rospy.Publisher("spatialDetectionTextMarkers", ImageMarkerArray, queue_size=1)

    def spatial_dections_callback(self, spatial_detection_array_msg):
        number_of_bounding_boxes = 0
        if(self.image is not None):
            bouding_box_image = self.image
            for detection in spatial_detection_array_msg.detections:
                bbox = detection.bbox

                detectionID = None
                score = -1.0
                label = None
                for result in detection.results:
                    if result.score > score:
                        detectionID = result.class_id
                        score = result.score


                #detectionID = detection.results[0].class_id
                #score = detection.results[0].score

                if(detectionID is not None):
                    position = detection.position
                    label = f'{self.labels[int(detectionID)]}, x: {round(position.x,3)}, y: {round(position.y,3)}, z: {round(position.z,3)}'
                    #print(label)

                    upper_left = (int(bbox.center.position.x - bbox.size_x/2),int(bbox.center.position.y - bbox.size_y/2))
                    lower_right = (int(bbox.center.position.x + bbox.size_x/2),int(bbox.center.position.y + bbox.size_y/2))

                    cv2.rectangle(bouding_box_image, upper_left, lower_right, (0,0,255), 1)

                    label_pos = (int(bbox.center.position.x - bbox.size_x/2),int(bbox.center.position.y + bbox.size_y/2 + 10))
                    
                    cv2.putText(bouding_box_image, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX,  0.5, (255, 0, 0), 1, cv2.LINE_AA) 

                    number_of_bounding_boxes = number_of_bounding_boxes + 1

            if number_of_bounding_boxes:
                image_message= self.bridge.cv2_to_imgmsg(bouding_box_image, encoding="passthrough")

                self.pubImageBoudingBoxes.publish(image_message)
                pass
      

    def image_callback(self, image_msg):

        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
 
        pass

def main(args=None):
    rclpy.init(args=args)

    publich_bounding_boxes = PublischBoundingBoxes()

    rclpy.spin(publich_bounding_boxes)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publich_bounding_boxes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
