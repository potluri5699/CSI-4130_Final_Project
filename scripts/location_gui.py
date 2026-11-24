#!/usr/bin/env python3
"""PyQt5 address-entry GUI that sends JSON route requests to the ROS 2 planner."""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QLineEdit, QPushButton,
                             QVBoxLayout, QHBoxLayout, QMessageBox, QGroupBox)


class RosSignals(QObject):
    status_received = pyqtSignal(str)


class GuiRosBridge(Node):
    def __init__(self, signals):
        super().__init__("osm_navigation_gui")
        self.publisher = self.create_publisher(String, "/route_request", 10)
        self.subscription = self.create_subscription(String, "/route_status",
                                                      lambda msg: signals.status_received.emit(msg.data), 10)

    def request_route(self, start, goal):
        msg = String()
        msg.data = json.dumps({"start": start, "goal": goal})
        self.publisher.publish(msg)


class NavigationWindow(QWidget):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("Oakland County OSM A* Navigation")
        self.setMinimumWidth(520)

        title = QLabel("OSM-Based Shortest-Distance Navigation")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        subtitle = QLabel("Enter two locations in Oakland County, Michigan.")

        self.start_input = QLineEdit("Troy, Michigan")
        self.goal_input = QLineEdit("Rochester Hills, Michigan")
        self.start_input.setPlaceholderText("Start address or place")
        self.goal_input.setPlaceholderText("Destination address or place")

        form = QGroupBox("Route locations")
        form_layout = QVBoxLayout()
        form_layout.addWidget(QLabel("Start location"))
        form_layout.addWidget(self.start_input)
        form_layout.addWidget(QLabel("Destination"))
        form_layout.addWidget(self.goal_input)
        form.setLayout(form_layout)

        self.submit_button = QPushButton("Find A* Route and Publish to RViz2")
        self.submit_button.clicked.connect(self.submit_request)
        self.status_label = QLabel("Status: Waiting for route request.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("padding: 8px; background: #eeeeee;")

        note = QLabel("The first request downloads and caches the Oakland County road graph.\n"
                      "Subsequent requests reuse the cached graph.")
        note.setWordWrap(True)

        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(form)
        layout.addWidget(self.submit_button)
        layout.addWidget(self.status_label)
        layout.addWidget(note)
        self.setLayout(layout)

    def submit_request(self):
        start = self.start_input.text().strip()
        goal = self.goal_input.text().strip()
        if not start or not goal:
            QMessageBox.warning(self, "Missing location", "Enter both a start and destination.")
            return
        self.bridge.request_route(start, goal)
        self.status_label.setText("Status: Request sent. Waiting for geocoding, map loading and A*...")
        self.submit_button.setEnabled(False)

    def update_status(self, message):
        self.status_label.setText("Status: " + message)
        if message.startswith("SUCCESS") or message.startswith("ERROR"):
            self.submit_button.setEnabled(True)


def main():
    rclpy.init()
    app = QApplication([])
    signals = RosSignals()
    bridge = GuiRosBridge(signals)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    window = NavigationWindow(bridge)
    signals.status_received.connect(window.update_status)
    window.show()
    try:
        app.exec_()
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
