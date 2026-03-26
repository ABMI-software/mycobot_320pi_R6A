import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_slider")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        
        # Initialisation du robot
        self.get_logger().info("Connexion au myCobot sur /dev/ttyAMA0...")
        self.mc = MyCobot("/dev/ttyAMA0", 115200)
        time.sleep(1)
        self.mc.power_on() # On s'assure que les moteurs sont allumés
        self.get_logger().info("Robot prêt !")

    def listener_callback(self, msg):
        # On ne garde que les 6 premières positions (les 6 axes du bras)
        if len(msg.position) >= 6:
            data_list = list(msg.position[:6])
            
            # Debug : affiche ce qu'on envoie réellement
            # self.get_logger().info(f"Envoi radians : {data_list}")
            
            # Envoi au robot physique
            self.mc.send_radians(data_list, 30)
        else:
            self.get_logger().warn("Message JointState trop court (moins de 6 joints)")

def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    try:
        rclpy.spin(slider_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        slider_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()