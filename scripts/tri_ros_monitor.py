"""Optional ROS 2 telemetry for the TCP sorting dashboard (no robot commands)."""
import json
import threading


class TriROSMonitor:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String

        self.context = Context()
        rclpy.init(context=self.context)
        self.source = Node('tri_reel_telemetrie', context=self.context)
        self.monitor = Node('tri_reel_monitor', context=self.context)
        self.message_type = String
        self.publishers = {}
        self.latest = {}
        self.subscriptions = []
        for name in ('codeurs', 'dream', 'yolo', 'etat'):
            topic = '/tri_reel/' + name
            self.publishers[name] = self.source.create_publisher(String, topic, 10)
            self.subscriptions.append(self.monitor.create_subscription(
                String, topic, lambda msg, key=name: self.latest.update({key: msg.data}), 10))
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.source)
        self.executor.add_node(self.monitor)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def publish(self, panneau, fenetre):
        with panneau.verrou:
            q = panneau.codeurs_q[-1].tolist() if panneau.codeurs_q else None
            dream = dict(panneau.kp_direct[-1]) if panneau.kp_direct else None
            yolo = dict(panneau.yolo_direct[-1]) if panneau.yolo_direct else None
        values = {'codeurs': {'angles_deg': q}, 'dream': dream, 'yolo': yolo,
                  'etat': {'cycles': len(panneau.cycles), 'journal': list(panneau.journal)[-5:]}}
        for name, value in values.items():
            if value is not None:
                message = self.message_type()
                message.data = json.dumps(value, ensure_ascii=False)
                self.publishers[name].publish(message)

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.source.destroy_node()
        self.monitor.destroy_node()
        self.context.shutdown()
