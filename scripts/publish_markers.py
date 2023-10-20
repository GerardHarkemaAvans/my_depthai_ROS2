#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray
from foxglove_msgs.msg import ImageMarkerArray
from visualization_msgs.msg import ImageMarker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

import json

class PublischMarkers(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        if 1:
            self.subSpatialDetection = self.create_subscription(
                SpatialDetectionArray,
                'color/yolov4_Spatial_detections',
                self.spatial_dections_callback,
                10)
            self.subSpatialDetection  # prevent unused variable warning


        self.pubMarkers = self.create_publisher(ImageMarkerArray, "spatialDetectionMarkers", 10)
            #    self.pubTextMarkers = rospy.Publisher("spatialDetectionTextMarkers", ImageMarkerArray, queue_size=1)



        #
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



    def spatial_dections_callback(self, spatial_detection_array_msg):
        markers = ImageMarkerArray()
        textMarker = ImageMarkerArray()
        i = 0
        for detection in spatial_detection_array_msg.detections:
            bbox = detection.bbox

            detectionID = None
            score = -1.0
            label = None
            for result in detection.results:
                if result.score > score:publisch_tf
                    detectionID = result.class_id
                    score = result.score


            #detectionID = detection.results[0].class_id
            #score = detection.results[0].score

            if(detectionID is not None):
                position = detection.position
                print(self.labels[detectionID])
                label = f'ID: {detectionID} \n score: {score} \n x: {position.x} \n y: {position.y} \n z: {position.z}'
                print(label)

                """ textMarker.markers.append(ImageMarker(
                        header=spatialMsgArray.header,
                        id=detectionID,
                        scale=2,        markers = ImageMarkerArray()
                    textMarker = ImageMarkerArray()
                        filled=1,
                        type=ImageMarker.TEXT,
                        outline_color=ColorRGBA(0, 1, 1, 1),
                        fill_color=ColorRGBA(b=255.0, a=0.2),
                        text=String(data=label),
                        position=Point(bbox.center.position.x - bbox.size_x/2, bbox.center.position.y + bbox.size_y/2, 0)
                    )
                    ) """
                print(bbox)
                point = Point()
                point.x = (bbox.center.position.x - bbox.size_x/2)
                point.y = (bbox.center.position.y + bbox.size_y/2)
                point.z = 0.0
                print(point)

                outline_color=ColorRGBA()
                outline_color.r = 0.0
                outline_color.g = 1.0
                outline_color.b = 1.0
                outline_color.a = 1.0
                
                fill_color=ColorRGBA()
                fill_color.b=255.0 
                fill_color.a=0.2
                
                points=[point]

                markers.markers.append(ImageMarker(
                        header=spatial_detection_array_msg.header,
                        id = int(detectionID),
                        scale=1.0,
                        filled=1,
                        type=ImageMarker.LINE_STRIP,
                        outline_color=outline_color,
                        fill_color=fill_color,
                        ##text=String(data=label), #???
                        points=points))
                
                i = i + 1
                print(".")
        if i:
            self.pubMarkers.publish(markers)
        # self.pubTextMarkers.publish(textMarker)       


def main(args=None):
    rclpy.init(args=args)

    publisch_markers = PublischMarkers()

    rclpy.spin(publisch_markers)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisch_markers.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
