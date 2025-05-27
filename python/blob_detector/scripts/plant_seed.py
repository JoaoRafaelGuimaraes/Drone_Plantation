#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from mrs_msgs.srv import Vec4
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

# Altitudes padrão
CRUISING_ALT = 2.5  # Altitude de voo
PLANTING_ALT = 1.0  # Altitude para plantar

def move_drone(x, y, z, yaw):
    rospy.wait_for_service('/uav1/control_manager/goto')
    goto = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
    try:
        result = goto([x, y, z, yaw])
        rospy.sleep(5)  # Dá tempo para o drone chegar
        return result.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao mover o drone: {e}")
        return False

def spawn_big_plant(name, x, y, z):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open('/usr/share/gazebo-11/models/big_plant/model.sdf', 'r') as f:
        model_xml = f.read()

    pose = Pose()
    pose.position = Point(x, y, z)
    pose.orientation = Quaternion(0, 0, 0, 1)

    try:
        spawn_model(model_name=name,
                    model_xml=model_xml,
                    robot_namespace='',
                    initial_pose=pose,
                    reference_frame='world')
        
        string = (f"Muda '{name}' plantada em ({x}, {y}, {z})")
        muda_pub.publish(name)
        rospy.loginfo(string)
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao plantar muda: {e}")

def plant_at_point(name, x, y, yaw):
    # Vai até o ponto na altitude de cruzeiro
    if not move_drone(x, y, CRUISING_ALT, yaw):
        rospy.logwarn("Falha ao chegar ao ponto de cruzeiro.")
        return

    # Desce até 1m do chão
    if not move_drone(x, y, PLANTING_ALT, yaw):
        rospy.logwarn("Falha ao descer para plantar.")
        return

    # Planta
    spawn_big_plant(name, x, y, 0.0)

    # Sobe de volta
    if not move_drone(x, y, CRUISING_ALT, yaw):
        rospy.logwarn("Falha ao subir após plantio.")


if __name__ == "__main__":
    rospy.init_node("drone_planter")
    muda_pub = rospy.Publisher("/muda_alert", String,queue_size=1)
    # Lista de pontos (adicione mais se quiser)
    points = [
    {"name": "Pequi",           "x": 5.0, "y": 2.0, "yaw": 0.0},
    {"name": "Cagaita",         "x": 6.0, "y": 2.0, "yaw": 0.0},
    {"name": "Baru",            "x": 7.0, "y": 2.0, "yaw": 0.0},
    {"name": "Murici",          "x": 8.0, "y": 2.0, "yaw": 0.0},
    {"name": "Mangaba",         "x": 9.0, "y": 2.0, "yaw": 0.0},
    {"name": "Araticum",        "x": 10.0, "y": 2.0, "yaw": 0.0},
    {"name": "Jenipapo",        "x": 11.0, "y": 2.0, "yaw": 0.0},
    {"name": "Bacupari",        "x": 12.0, "y": 2.0, "yaw": 0.0},
    {"name": "Guabiroba",       "x": 13.0, "y": 2.0, "yaw": 0.0},
    {"name": "Gabiroba-do-campo","x": 14.0, "y": 2.0, "yaw": 0.0},
]
    for p in points:
        plant_at_point(p["name"], p["x"], p["y"], p["yaw"])

