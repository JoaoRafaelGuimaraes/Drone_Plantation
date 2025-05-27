#!/usr/bin/env python3
import time
import rospy
import simplekml
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class KMLGenerator:
    def __init__(self):
        rospy.init_node("kml_generator", anonymous=True)

        # Assina os tópicos
        rospy.Subscriber("/muda_alert", String, self.points_callback)
        rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, self.update_pos)

        self.points = []  # Lista de tuplas: (nome_muda, lat, long)
        self.lat = None 
        self.long = None 

        rospy.on_shutdown(self.save_kml)

        rospy.loginfo("KML Generator Node iniciado.")
        rospy.spin()

    def update_pos(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude

    def points_callback(self, msg):
        if self.lat is None or self.long is None:
            rospy.logwarn("Coordenadas GPS não disponíveis no momento do plantio.")
            return

        nome_muda = msg.data.strip()
        self.points.append((nome_muda, self.lat, self.long))
        rospy.loginfo(f"Registrado: {nome_muda} em ({self.lat}, {self.long})")

    def save_kml(self):
        if not self.points:
            rospy.logwarn("Nenhum ponto registrado. KML não será gerado.")
            return

        kml = simplekml.Kml()

        for nome_muda, lat, lon in self.points:
            pnt = kml.newpoint(coords=[(lon, lat)])
            pnt.name = nome_muda
            pnt.description = f"Muda plantada: {nome_muda}\nLatitude: {lat:.6f}, Longitude: {lon:.6f}"

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"/root/KMLs/kml_{timestamp}.kml"
        kml.save(filename)
        rospy.loginfo(f"KML salvo como {filename}")

if __name__ == "__main__":
    try:
        KMLGenerator()
    except rospy.ROSInterruptException:
        pass
