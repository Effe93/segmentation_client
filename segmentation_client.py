#! /usr/bin/env python


import os

import cv2

import roslib
roslib.load_manifest('ipa_room_segmentation')
import rospy
from rospkg import RosPack
import actionlib
import os, sys


from cv_bridge import CvBridge

from geometry_msgs.msg import Pose
from ipa_building_msgs.msg import MapSegmentationAction, MapSegmentationGoal
import glob

algo = [1,2,3]
maps = glob.glob('/home/matthias/Segmentierungstests/easyflats/*.png')



if __name__ == '__main__':
    rospy.init_node('segmentation_client')
    client = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
    client.wait_for_server()
    rospack = RosPack()

    for j in maps:
	#print(os.path.basename(j))
        for i in algo:
            # Load an image.
            image_path = os.path.join(str(j))
            cv_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            bw_image = cv2.threshold(cv_image, 254, 255, cv2.THRESH_BINARY)[1]

            cv_bridge = CvBridge()

            # Compose the segmentation action.
            goal = MapSegmentationGoal()
            # Fill in the content:
            goal.input_map = cv_bridge.cv2_to_imgmsg(bw_image, 'mono8')
            goal.map_resolution = 0.05
            goal.map_origin = Pose()
            goal.return_format_in_pixel = True
            goal.return_format_in_meter = True
            goal.robot_radius = 0.15
            goal.room_segmentation_algorithm = i

            client.send_goal(goal)
             #print("goal")
            client.wait_for_result()
             #print("waitforresult")
            res = client.get_result()
             #print("result")
            """print(res.segmented_map)"""
            bridge = CvBridge()
            """img=res.segmented_map"""
 	    # print("res")
            buidl = bridge.imgmsg_to_cv2(res.segmented_map)
            """cv2.imshow("hallo",img)
            cv2.waitKey(0)
            os.makedirs("/home/matthias/img", 0777)"""
	    #print("res")
            #cv2.imwrite(os.path.join('/home/matthias/img' , str(j)+"Algo"+str(i)+".png"), img)
            path = os.path.join('/home/matthias/Segmentierungstests/easyflatsres' , '{m}_algo_{a}.png'.format(m=os.path.basename(j), a=i))
            #print('writing {p}'.format(p=path))
            ret = cv2.imwrite(path, buidl)
            #print('returned {r}'.format(r=ret))

