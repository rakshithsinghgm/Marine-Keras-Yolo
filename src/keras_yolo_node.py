#!/usr/bin/env python
import rospy
import darknet
import numpy as np
import cv2

import sys
import argparse
from yolo import YOLO, detect_video
from PIL import Image

from sensor_msgs.msg import Image, CompressedImage
from asv_perception_common.msg import Classification, ClassificationArray
import asv_perception_utils as utils

class darknet_node(object):

    def __init__(self):

        self.node_name = rospy.get_name()

        # publisher of classification array
        self.pub = rospy.Publisher( "~output", ClassificationArray, queue_size=1)

        # image publisher for visualization/debugging
        self.pub_img = rospy.Publisher( "~image", Image, queue_size=1)

        # subscribers
        self.sub = rospy.Subscriber( "~input", CompressedImage, self.processImage, queue_size=1, buff_size=2**24 )
        

    def processImage(self, image_msg):

        # no subscribers, no work
        if self.pub.get_num_connections() <= 0 and self.pub_img.get_num_connections() <= 0:
            return

        rospy.logdebug( 'Processing img with timestamp secs=%d, nsecs=%d', image_msg.header.stamp.secs, image_msg.header.stamp.nsecs )

        # darknet requires rgb image in proper shape.  we need to resize, and then convert resulting bounding boxes to proper shape
        img = utils.convert_ros_msg_to_cv2( image_msg, 'rgb8' )

        # Object detection code

        r_image = yolo.detect_image(image)

        #self.pub.publish( msg )

        self.publishImage(r_image)

    def publishImage( self, img):
        """
        Publish the annotated image for visualization/debugging
        """
        # no subscribers, no work
        if self.pub_img.get_num_connections() <= 0:
            return
    
        # publish
        msg = utils.convert_cv2_to_ros_msg( img, 'rgb8' )
        msg.header = clsMsg.header # match timestamp
        self.pub_img.publish( msg )

if __name__ == "__main__":
    try:
        rospy.init_node("Keras_Yolo")
        n = darknet_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
