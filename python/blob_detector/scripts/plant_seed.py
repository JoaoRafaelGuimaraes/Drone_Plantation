#!/usr/bin/env python3

import rospy
import math
from threading import Lock
import os
from gazebo_msgs.srv import SpawnModel
from mrs_msgs.srv import Vec4, PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

CRUISING_ALT = 2.5
PLANTING_ALT = 1.0
TOLERANCE   = 1 # m

class PlanterNode:
    def __init__(self):
        rospy.init_node("drone_planter", anonymous=True)    
        self.muda_pub = rospy.Publisher("/muda_alert", String,queue_size=1)
        self.points = [
            {"name":"Pequi",             "x":5.0,  "y":2.0,  "yaw":0.0},
            {"name":"Cagaita",           "x":9.0,  "y":4.0,  "yaw":0.0},
            {"name":"Baru",              "x":5.0,  "y":-5.0,  "yaw":0.0},
            {"name":"Murici",            "x":12.0,  "y":7.0,  "yaw":0.0},
            {"name":"Mangaba",           "x":15.0,  "y":9.0,  "yaw":0.0},
            {"name":"Araticum",          "x":6.0, "y":4.0,  "yaw":0.0},
            {"name":"Jenipapo",          "x":11.0, "y":10.0,  "yaw":0.0},
            {"name":"Bacupari",          "x":4.0, "y":11.0,  "yaw":0.0},
            {"name":"Guabiroba",         "x":2.0, "y":3.0,  "yaw":0.0},
            {"name":"Gabiroba-do-campo", "x":3.0, "y":4.0,  "yaw":0.0},
        ]

        self.current_idx = 0
        self.planting    = False
        self.lock        = Lock()

        # serviços
        rospy.wait_for_service('/uav1/trajectory_generation/path')
        self.path_srv = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)

        rospy.wait_for_service('/uav1/control_manager/goto')
        self.goto_srv = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # agora assinamos o odom_main, que mostra a posição real do drone
        self.sub_odom = rospy.Subscriber(
            '/uav1/estimation_manager/odom_main',
            Odometry,
            self.odom_callback,
            queue_size=1
        )

        # aguarde tudo subir e então envie o plano completo
        rospy.sleep(1.0)
        self.send_trajectory(self.points, prepend_current=False)

    def send_trajectory(self, pts, prepend_current):
        req = PathSrvRequest()
        req.path.header.frame_id = ""
        req.path.header.stamp = rospy.Time.now()
        req.path.use_heading             = True
        req.path.fly_now                 = True
        req.path.dont_prepend_current_state = prepend_current
        req.path.max_execution_time      = 60.0
        req.path.max_deviation_from_path = 0.0

        for p in pts:
            r = Reference()
            r.position.x = p["x"]
            r.position.y = p["y"]
            r.position.z = CRUISING_ALT
            r.heading    = p["yaw"]
            req.path.points.append(r)
            rospy.loginfo(f"[PLAN] → {p['name']} at ({p['x']:.1f},{p['y']:.1f})")

        resp = self.path_srv(req)
        if not resp.success:
            rospy.logerr("[PLAN] failed: "+resp.message)
        else:
            rospy.loginfo("[PLAN] trajectory sent")

    
    def odom_callback(self, msg: Odometry):
        with self.lock:
            if self.planting or self.current_idx >= len(self.points):
                return

            pos = msg.pose.pose.position
            pt  = self.points[self.current_idx]
            dist = math.hypot(pos.x - pt["x"], pos.y - pt["y"])

            if dist < TOLERANCE:
                rospy.loginfo(f"[REACHED] {pt['name']} (idx={self.current_idx})")
                self.planting = True

                # 1) FORA DA TRAJETÓRIA: faça um goto instantâneo para bloquear
                # self.cancel_trajectory()
                rospy.sleep(0.1)
                self.goto([pt["x"], pt["y"], CRUISING_ALT, pt["yaw"]])
                rospy.sleep(0.5)

                # 2) plante
                self.do_plant(pt)

                self.current_idx += 1
                # 3) reenvie o restante da rota
                if self.current_idx < len(self.points):
                    rem = self.points[self.current_idx:]
                    print(rem)
                    rospy.sleep(0.5)
                    self.send_trajectory(rem, prepend_current=False)
                else:
                    rospy.loginfo("[FINISH] todas as mudas plantadas")

                self.planting = False

    def do_plant(self, pt):
        name, x, y, yaw = pt["name"], pt["x"], pt["y"], pt["yaw"]

        rospy.loginfo(f"[PLANT] descending for {name}")
        self.goto([x, y, PLANTING_ALT, yaw])

        rospy.loginfo(f"[PLANT] spawning '{name}'")
        self.spawn_plant(name, x, y)

        self.muda_pub.publish(name) #publica o alert
        rospy.loginfo(f"[PLANT] ascending after {name}")
        self.goto([x, y, CRUISING_ALT, yaw])

    def goto(self, vec4):
        try:
            self.goto_srv(vec4)
            rospy.sleep(4.0)
        except rospy.ServiceException as e:
            rospy.logwarn(f"[GOTO] failed: {e}")

    def spawn_plant(self, name, x, y):
        if not hasattr(self, "model_xml"):
            with open('/usr/share/gazebo-11/models/big_plant/model.sdf','r') as f:
                self.model_xml = f.read()

        pose = Pose()
        pose.position = Point(x, y, 0.0)
        pose.orientation = Quaternion(0,0,0,1)
        try:
            self.spawn_srv(
                model_name=name,
                model_xml=self.model_xml,
                robot_namespace='',
                initial_pose=pose,
                reference_frame='world'
            )
            rospy.loginfo(f"[SPAWN] planted '{name}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"[SPAWN] failed: {e}")

if __name__ == "__main__":
    try:
        PlanterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
