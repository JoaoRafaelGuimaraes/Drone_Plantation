#!/usr/bin/env python3

import rospy
from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference

class Goto:

    def __init__(self):
        rospy.init_node('plan_path', anonymous=True)

        # wait for the path‐generation service to be available
        rospy.wait_for_service('/uav1/trajectory_generation/path')
        self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)

        # define your ten key waypoints
        points = [
            {"name": "Pequi",            "x":  5.0, "y": 2.0, "yaw": 0.0},
            {"name": "Cagaita",          "x":  6.0, "y": 2.0, "yaw": 0.0},
            {"name": "Baru",             "x":  7.0, "y": 2.0, "yaw": 0.0},
            {"name": "Murici",           "x":  8.0, "y": 2.0, "yaw": 0.0},
            {"name": "Mangaba",          "x":  9.0, "y": 2.0, "yaw": 0.0},
            {"name": "Araticum",         "x": 10.0, "y": 2.0, "yaw": 0.0},
            {"name": "Jenipapo",         "x": 11.0, "y": 2.0, "yaw": 0.0},
            {"name": "Bacupari",         "x": 12.0, "y": 2.0, "yaw": 0.0},
            {"name": "Guabiroba",        "x": 13.0, "y": 2.0, "yaw": 0.0},
            {"name": "Gabiroba-do-campo","x": 14.0, "y": 2.0, "yaw": 0.0},
        ]

        # build the PathSrvRequest
        req = PathSrvRequest()
        req.path.header.frame_id = ""
        req.path.header.stamp = rospy.Time.now()

        req.path.use_heading = True
        req.path.fly_now     = True
        req.path.max_execution_time       = 30.0
        req.path.max_deviation_from_path  = 0.0
        req.path.dont_prepend_current_state = False

        # append each named point at altitude z=5.0
        for p in points:
            ref = Reference()
            ref.position.x = p["x"]
            ref.position.y = p["y"]
            ref.position.z = 5.0
            ref.heading    = p["yaw"]
            req.path.points.append(ref)
            rospy.loginfo(f"Added waypoint '{p['name']}' at ({p['x']}, {p['y']}, 5.0")

        # call the service and handle response
        try:
            resp = self.sc_path(req)
            if resp.success:
                rospy.loginfo("Path successfully planned and sent.")
            else:
                rospy.logerr("Path planning failed: " + resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        Goto()
    except rospy.ROSInterruptException:
        pass
