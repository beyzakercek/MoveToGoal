#!/usr/bin/env python
# chmod u+x catkin_ws/src/beginner_msgsrv/src/distanceservice.py

import rospy
from math import pow,sqrt
from beginner_msgsrv.srv import euclideandistance, euclideandistanceResponse
        
def euclidean_distance_func(req):
	return euclideandistanceResponse(sqrt(pow((req.goalpose_X-req.selfpose_X),2.0)+pow(( req.goalpose_Y-req.selfpose_Y),2.0)))
	
def movetogoal_server():
	rospy.init_node("euclideandistanceserver")
	rospy.Service('euclideanDistance', euclideandistance, euclidean_distance_func)
	rospy.spin()

if __name__ == "__main__":
	movetogoal_server()
