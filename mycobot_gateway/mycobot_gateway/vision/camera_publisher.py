#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Publisher Node - Runs on Tour (PC)
Captures video from USB camera and publishes to ROS2 topic.
"""

import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def set_manual_exposure(video_index: int, exposure: int):
    """Pin exposure/gain/brightness via v4l2-ctl — same recipe as
    training/capture_real_3cam.py's set_arducam_exposure(). The Arducam's
    auto-exposure blows out the image under this rig's lighting (washed out,
    "très lumineux"); a fixed manual exposure with gain=0, brightness=0
    reproduces the session4 reference look the model was trained against.
    auto_exposure must go to manual FIRST, otherwise exposure_time_absolute
    is inert. Re-tune if the room lighting changes.
    """
    dev = f'/dev/video{int(video_index)}'
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'auto_exposure=1'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                    f'exposure_time_absolute={exposure},gain=0,brightness=0'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def set_auto_exposure(video_index: int):
    """Force l'auto-exposition (auto_exposure=3) + gain auto. Les contrôles
    v4l2 PERSISTENT dans la caméra entre les processus : si un run précédent a
    épinglé une exposition manuelle sombre sur ce device (ex. la SVPRO qui a
    hérité du 75 de l'arducam lors d'un conflit d'index), la laisser telle
    quelle la garde sombre. On la remet donc explicitement en auto ici."""
    dev = f'/dev/video{int(video_index)}'
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'auto_exposure=3'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'gain=100,brightness=0'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def set_manual_focus(video_index: int, focus: int):
    """Verrouille le focus + rendu image — même recette que
    training/capture_real_3cam.py's set_svpro_focus() (SVPRO). L'objectif à
    focale variable de la SVPRO doit être épinglé (focus_absolute=90 = net sur
    le poignet/LED) sinon l'autofocus pompe entre les poses et l'image plonge
    dans la zone catastrophiquement floue. autofocus OFF d'abord, sinon
    focus_absolute est inerte. sharpness=0, contrast=1 = rendu session9.
    """
    dev = f'/dev/video{int(video_index)}'
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'focus_automatic_continuous=0'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                    f'focus_absolute={focus},sharpness=0,contrast=1'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class CameraPublisher(Node):
    """
    Camera node running on Tour (PC).
    Publishes camera images for processing by marker_detector.
    """

    def __init__(self):
        super().__init__('camera_publisher_tour')

        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        # -1 disables the manual-exposure fix (e.g. for a camera without a
        # v4l2 exposure control). 75 matches capture_real_3cam.py's tuned
        # default for the arducam under this rig's lighting.
        self.declare_parameter('manual_exposure', 75)
        # -1 désactive le verrou de focus (arducam : focale fixe). 90 = réglage
        # SVPRO de capture_real_3cam.py (net sur poignet/LED). Exclusif de
        # l'exposition : la SVPRO utilise focus (pas d'expo), l'arducam expo.
        self.declare_parameter('manual_focus', -1)
        # Topic de sortie — paramétrable pour lancer plusieurs instances (une
        # par caméra) sans collision. Défaut = nom legacy mono-caméra.
        self.declare_parameter('output_topic', 'camera/image_raw')

        # Get parameters
        camera_index = self.get_parameter('camera_index').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        manual_exposure = self.get_parameter('manual_exposure').value
        manual_focus = self.get_parameter('manual_focus').value
        output_topic = self.get_parameter('output_topic').value

        # Publisher
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.bridge = CvBridge()

        # Camera setup
        self.cap, opened_index = self.find_camera(camera_index)

        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # petit tampon (cf. 3cam.py)

            if manual_exposure is not None and manual_exposure >= 0:
                # Let the stream settle first — a v4l2 control set before the
                # first read() can get silently reset by the driver on stream
                # start (same ordering as capture_real_3cam.py).
                time.sleep(2)
                for _ in range(5):
                    self.cap.read()
                set_manual_exposure(opened_index, manual_exposure)
                for _ in range(5):
                    self.cap.read()
                self.get_logger().info(f"🔆 Exposition manuelle fixée : {manual_exposure}")
            else:
                # Pas d'exposition manuelle (ex. SVPRO) : forcer l'auto pour
                # effacer tout réglage manuel sombre coincé dans le device.
                time.sleep(2)
                for _ in range(5):
                    self.cap.read()
                set_auto_exposure(opened_index)
                for _ in range(5):
                    self.cap.read()
                self.get_logger().info("🔆 Auto-exposition forcée (device remis à zéro)")

            if manual_focus is not None and manual_focus >= 0:
                time.sleep(2)
                for _ in range(5):
                    self.cap.read()
                set_manual_focus(opened_index, manual_focus)
                for _ in range(5):
                    self.cap.read()
                self.get_logger().info(f"🎯 Focus manuel fixé (SVPRO) : {manual_focus}")

            # Timer for capture
            timer_period = 1.0 / fps
            self.timer = self.create_timer(timer_period, self.timer_callback)

            self.get_logger().info(f"📷 Camera initialized at index {opened_index}")
            self.get_logger().info(f"📐 Resolution: {frame_width}x{frame_height} @ {fps}fps")
        else:
            self.get_logger().error("❌ CRITICAL: No camera found!")

    def find_camera(self, preferred_index):
        """Find a working camera. Returns (cap, index) or (None, None).

        Force le backend V4L2 (cv2.CAP_V4L2), comme capture_real_3cam.py : le
        backend GStreamer par défaut ne pilote pas correctement l'auto-exposition
        de certaines caméras (SVPRO sort très sombre). En V4L2, l'auto-exposition
        se comporte comme à la capture 3cam (image propre et bien exposée)."""
        # Try preferred index first
        cap = cv2.VideoCapture(preferred_index, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                self.get_logger().info(f"✅ Camera found at index {preferred_index}")
                return cap, preferred_index
            cap.release()

        # Search other indices
        for index in range(6):
            if index == preferred_index:
                continue
            self.get_logger().info(f"🔍 Testing camera index {index}...")
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info(f"✅ Camera found at index {index}")
                    return cap, index
                cap.release()

        return None, None

    def timer_callback(self):
        """Capture and publish frame"""
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("⚠️ Dropped frame")

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
