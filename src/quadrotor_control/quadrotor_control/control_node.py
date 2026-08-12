import sys
import os
import threading
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, QGridLayout,
                             QVBoxLayout, QHBoxLayout, QTextEdit, QLabel, QGroupBox)
from PyQt5.QtCore import pyqtSignal, QObject, QTimer

# Offline Vosk Speech Recognition Imports
try:
    import vosk
    import sounddevice as sd
    import queue
    import json
    VOSK_AVAILABLE = True
except ImportError:
    VOSK_AVAILABLE = False


def quaternion_to_euler(x, y, z, w):
    """
    Converts quaternion (x, y, z, w) to euler angles (roll, pitch, yaw) in radians.
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SignalBridge(QObject):
    log_signal = pyqtSignal(str)
    voice_status_signal = pyqtSignal(str)
    # x, y, z, qx, qy, qz, qw, vx, vy, vz, wz
    telemetry_signal = pyqtSignal(float, float, float, float, float, float, float, float, float, float, float)


class ROS2Bridge(Node):
    def __init__(self, gui_bridge):
        super().__init__('quadrotor_control_node')
        self.gui_bridge = gui_bridge
        self.publisher = self.create_publisher(Twist, '/X3/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/model/X3/odometry',
            self.odom_callback,
            10
        )
        self.get_logger().info("ROS2 Quadrotor Control Node Initialized.")

    def publish_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def odom_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wz = msg.twist.twist.angular.z

        # Emit telemetry safely to GUI thread
        self.gui_bridge.telemetry_signal.emit(x, y, z, qx, qy, qz, qw, vx, vy, vz, wz)


class DroneGUI(QWidget):
    def __init__(self, ros_node, signal_bridge):
        super().__init__()
        self.ros_node = ros_node
        self.bridge = signal_bridge

        self.bridge.log_signal.connect(self.log_message)
        self.bridge.voice_status_signal.connect(self.update_voice_status)
        self.bridge.telemetry_signal.connect(self.handle_telemetry)

        # Control states
        self.control_mode = "MANUAL"  # Default mode
        self.listening = False
        self.active_voice_thread = None
        self.last_odom_time = 0.0

        # Continuous movement variables
        self.active_cmd = {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_z': 0.0
        }

        # Publish timer for continuous manual movement (10Hz)
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.send_active_cmd)
        self.publish_timer.start(100)

        # Voice command timer for auto-stopping
        self.voice_stop_timer = QTimer()
        self.voice_stop_timer.setSingleShot(True)
        self.voice_stop_timer.timeout.connect(self.voice_timeout_stop)

        # Watchdog timer for connection status (2Hz)
        self.watchdog_timer = QTimer()
        self.watchdog_timer.timeout.connect(self.check_connection)
        self.watchdog_timer.start(500)

        self.init_ui()

        # Try to find Vosk model path
        self.model_path = "/opt/vosk-model"
        if not os.path.exists(self.model_path):
            self.model_path = os.path.join(os.path.expanduser("~"), ".cache/vosk/vosk-model-small-en-us-0.15")

        # Set default control mode to Manual on init
        self.set_control_mode("MANUAL")

    def init_ui(self):
        self.setWindowTitle("X3 Quadcopter Controller & Telemetry Dashboard")
        self.resize(700, 750)

        main_layout = QVBoxLayout()

        # --- Section 1: Dual-Mode Control Selection ---
        mode_group = QGroupBox("System Control Mode Selection")
        mode_layout = QHBoxLayout()

        self.btn_mode_manual = QPushButton("MANUAL MODE")
        self.btn_mode_manual.clicked.connect(lambda: self.set_control_mode("MANUAL"))

        self.btn_mode_voice = QPushButton("VOICE MODE")
        self.btn_mode_voice.clicked.connect(lambda: self.set_control_mode("VOICE"))

        self.lbl_active_mode = QLabel("Active Mode: MANUAL")
        self.lbl_active_mode.setStyleSheet("font-weight: bold; font-size: 13px; color: #1976d2; padding: 5px;")

        mode_layout.addWidget(self.btn_mode_manual)
        mode_layout.addWidget(self.btn_mode_voice)
        mode_layout.addWidget(self.lbl_active_mode)
        mode_group.setLayout(mode_layout)
        main_layout.addWidget(mode_group)

        # --- Section 2: Real-time Telemetry Dashboard ---
        telemetry_group = QGroupBox("Real-time Telemetry Dashboard")
        telemetry_layout = QVBoxLayout()

        # Connection and Status
        status_row = QHBoxLayout()
        lbl_conn_title = QLabel("System Status: ")
        self.lbl_conn_status = QLabel("DISCONNECTED")
        self.lbl_conn_status.setStyleSheet("color: #d32f2f; font-weight: bold; font-size: 14px;")
        status_row.addWidget(lbl_conn_title)
        status_row.addWidget(self.lbl_conn_status)
        status_row.addStretch()
        telemetry_layout.addLayout(status_row)

        # Labels for telemetry data
        self.lbl_pos = QLabel("Position: X: 0.00 m | Y: 0.00 m | Z: 0.00 m")
        self.lbl_pos.setStyleSheet("font-size: 13px; padding: 3px;")

        self.lbl_ori = QLabel("Orientation: Roll: 0.0° | Pitch: 0.0° | Yaw: 0.0°")
        self.lbl_ori.setStyleSheet("font-size: 13px; padding: 3px;")

        self.lbl_vel = QLabel("Velocities: Vx: 0.00 m/s | Vy: 0.00 m/s | Vz: 0.00 m/s | Wz: 0.0°/s")
        self.lbl_vel.setStyleSheet("font-size: 13px; padding: 3px;")

        telemetry_layout.addWidget(self.lbl_pos)
        telemetry_layout.addWidget(self.lbl_ori)
        telemetry_layout.addWidget(self.lbl_vel)
        telemetry_group.setLayout(telemetry_layout)
        main_layout.addWidget(telemetry_group)

        # --- Section 3: Voice Control Section ---
        voice_group = QGroupBox("Voice Control (Active in Voice Mode)")
        voice_layout = QHBoxLayout()

        self.btn_voice = QPushButton("Start Listening")
        self.btn_voice.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")
        self.btn_voice.clicked.connect(self.toggle_voice_control)

        self.voice_status_label = QLabel("Status: Idle")
        voice_layout.addWidget(self.btn_voice)
        voice_layout.addWidget(self.voice_status_label)
        voice_group.setLayout(voice_layout)
        main_layout.addWidget(voice_group)

        # --- Section 4: Manual Control Section ---
        manual_group = QGroupBox("Manual Controls (Active in Manual Mode)")
        grid = QGridLayout()

        self.btn_up = QPushButton("Ascend (Up)")
        self.btn_down = QPushButton("Descend (Down)")
        self.btn_fwd = QPushButton("Forward")
        self.btn_back = QPushButton("Backward")
        self.btn_left = QPushButton("Left")
        self.btn_right = QPushButton("Right")
        self.btn_yaw_left = QPushButton("Rotate Left")
        self.btn_yaw_right = QPushButton("Rotate Right")

        # Emergency stop should always be active regardless of active mode for ultimate safety
        self.btn_stop = QPushButton("EMERGENCY STOP / HOVER")
        self.btn_stop.setStyleSheet("background-color: #c62828; color: white; font-weight: bold; padding: 10px;")

        # Set up pressed and released behaviors for continuous movement
        self.btn_up.pressed.connect(lambda: self.set_direction(linear_z=0.5))
        self.btn_up.released.connect(self.clear_direction)

        self.btn_down.pressed.connect(lambda: self.set_direction(linear_z=-0.5))
        self.btn_down.released.connect(self.clear_direction)

        self.btn_fwd.pressed.connect(lambda: self.set_direction(linear_x=0.5))
        self.btn_fwd.released.connect(self.clear_direction)

        self.btn_back.pressed.connect(lambda: self.set_direction(linear_x=-0.5))
        self.btn_back.released.connect(self.clear_direction)

        self.btn_left.pressed.connect(lambda: self.set_direction(linear_y=0.5))
        self.btn_left.released.connect(self.clear_direction)

        self.btn_right.pressed.connect(lambda: self.set_direction(linear_y=-0.5))
        self.btn_right.released.connect(self.clear_direction)

        self.btn_yaw_left.pressed.connect(lambda: self.set_direction(angular_z=0.5))
        self.btn_yaw_left.released.connect(self.clear_direction)

        self.btn_yaw_right.pressed.connect(lambda: self.set_direction(angular_z=-0.5))
        self.btn_yaw_right.released.connect(self.clear_direction)

        self.btn_stop.clicked.connect(self.emergency_stop)

        grid.addWidget(self.btn_up, 0, 0)
        grid.addWidget(self.btn_fwd, 0, 1)
        grid.addWidget(self.btn_yaw_left, 0, 2)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_down, 2, 0)
        grid.addWidget(self.btn_back, 2, 1)
        grid.addWidget(self.btn_yaw_right, 2, 2)

        # Keep a list of manual control buttons to enable/disable them
        self.manual_buttons = [
            self.btn_up, self.btn_down, self.btn_fwd, self.btn_back,
            self.btn_left, self.btn_right, self.btn_yaw_left, self.btn_yaw_right
        ]

        manual_group.setLayout(grid)
        main_layout.addWidget(manual_group)

        # --- Section 5: Log Console ---
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        main_layout.addWidget(self.log_box)

        self.setLayout(main_layout)
        self.log_message("GUI initialized. Seamless Mode selection at the top.")

    def log_message(self, message):
        self.log_box.append(message)

    def update_voice_status(self, status):
        self.voice_status_label.setText(f"Status: {status}")

    def set_control_mode(self, mode):
        """
        Transitions cleanly and seamlessly between MANUAL and VOICE control modes.
        """
        if mode == "MANUAL":
            self.control_mode = "MANUAL"
            self.lbl_active_mode.setText("Active Mode: MANUAL")
            self.lbl_active_mode.setStyleSheet("font-weight: bold; font-size: 13px; color: #1976d2; padding: 5px;")

            # Styling Mode Buttons
            self.btn_mode_manual.setStyleSheet("background-color: #1976d2; color: white; font-weight: bold; padding: 8px;")
            self.btn_mode_voice.setStyleSheet("background-color: #e0e0e0; color: black; padding: 8px;")

            # Enable manual movement controls
            for btn in self.manual_buttons:
                btn.setEnabled(True)

            # Deactivate voice listening automatically
            if self.listening:
                self.listening = False
                self.btn_voice.setText("Start Listening")
                self.btn_voice.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")
                self.voice_status_label.setText("Status: Disabled in Manual Mode")

            self.log_message("System Control Mode transitioned to: MANUAL")

        elif mode == "VOICE":
            self.control_mode = "VOICE"
            self.lbl_active_mode.setText("Active Mode: VOICE")
            self.lbl_active_mode.setStyleSheet("font-weight: bold; font-size: 13px; color: #2e7d32; padding: 5px;")

            # Styling Mode Buttons
            self.btn_mode_manual.setStyleSheet("background-color: #e0e0e0; color: black; padding: 8px;")
            self.btn_mode_voice.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 8px;")

            # Disable manual movement controls (safety Emergency Stop button is still kept enabled!)
            for btn in self.manual_buttons:
                btn.setEnabled(False)

            # Automatically start voice listening for maximum seamless user experience
            if not self.listening:
                if not VOSK_AVAILABLE:
                    self.log_message("Error: Vosk or sounddevice package is not installed. Voice control unavailable.")
                    return
                self.listening = True
                self.btn_voice.setText("Stop Listening")
                self.btn_voice.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 10px;")
                self.voice_status_label.setText("Status: Listening (Offline)...")
                self.active_voice_thread = threading.Thread(target=self.vosk_listen_loop, daemon=True)
                self.active_voice_thread.start()

            self.log_message("System Control Mode transitioned to: VOICE")

    def handle_telemetry(self, x, y, z, qx, qy, qz, qw, vx, vy, vz, wz):
        """
        Receives real-time telemetry from ROS2, converts quaternion to Euler angles,
        and thread-safely updates display labels.
        """
        self.last_odom_time = time.time()

        # Quaternion to Euler (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        # Update Telemetry Dashboard
        self.lbl_pos.setText(f"Position: X: {x:.2f} m | Y: {y:.2f} m | Z: {z:.2f} m")
        self.lbl_ori.setText(f"Orientation: Roll: {math.degrees(roll):.1f}° | Pitch: {math.degrees(pitch):.1f}° | Yaw: {math.degrees(yaw):.1f}°")
        self.lbl_vel.setText(f"Velocities: Vx: {vx:.2f} m/s | Vy: {vy:.2f} m/s | Vz: {vz:.2f} m/s | Wz: {math.degrees(wz):.1f}°/s")

    def check_connection(self):
        """
        Watchdog check to see if we are actively receiving odometry messages.
        """
        if time.time() - self.last_odom_time < 2.0:
            self.lbl_conn_status.setText("CONNECTED")
            self.lbl_conn_status.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 14px;")
        else:
            self.lbl_conn_status.setText("DISCONNECTED")
            self.lbl_conn_status.setStyleSheet("color: #d32f2f; font-weight: bold; font-size: 14px;")

    def set_direction(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        # Double safety check: ignore manual commands if not in Manual Mode
        if self.control_mode != "MANUAL":
            return
        self.active_cmd['linear_x'] = linear_x
        self.active_cmd['linear_y'] = linear_y
        self.active_cmd['linear_z'] = linear_z
        self.active_cmd['angular_z'] = angular_z

    def clear_direction(self):
        # Double safety check: ignore manual commands if not in Manual Mode
        if self.control_mode != "MANUAL":
            return
        self.active_cmd['linear_x'] = 0.0
        self.active_cmd['linear_y'] = 0.0
        self.active_cmd['linear_z'] = 0.0
        self.active_cmd['angular_z'] = 0.0
        self.ros_node.publish_twist(0, 0, 0, 0)

    def emergency_stop(self):
        # Emergency stop can be called any time for ultimate safety
        self.active_cmd['linear_x'] = 0.0
        self.active_cmd['linear_y'] = 0.0
        self.active_cmd['linear_z'] = 0.0
        self.active_cmd['angular_z'] = 0.0
        self.ros_node.publish_twist(0, 0, 0, 0)
        self.log_message("EMERGENCY STOP / HOVER SENT")

    def send_active_cmd(self):
        # Publish continuous manual commands only if in Manual Mode and a direction is active
        if self.control_mode == "MANUAL" and any(v != 0.0 for v in self.active_cmd.values()):
            self.ros_node.publish_twist(
                self.active_cmd['linear_x'],
                self.active_cmd['linear_y'],
                self.active_cmd['linear_z'],
                self.active_cmd['angular_z']
            )

    def voice_timeout_stop(self):
        # Stop voice command and hover
        self.ros_node.publish_twist(0.0, 0.0, 0.0, 0.0)
        self.bridge.log_signal.emit("Voice command duration completed. Hovering.")

    def toggle_voice_control(self):
        """
        Manually toggle microphone listening within Voice mode or toggle voice mode.
        """
        if self.control_mode != "VOICE":
            # Seamless transition to Voice mode
            self.set_control_mode("VOICE")
            return

        if not self.listening:
            if not VOSK_AVAILABLE:
                self.log_message("Error: Vosk or sounddevice package is not installed. Voice control unavailable.")
                return
            self.listening = True
            self.btn_voice.setText("Stop Listening")
            self.btn_voice.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 10px;")
            self.bridge.voice_status_signal.emit("Listening (Offline)...")
            self.active_voice_thread = threading.Thread(target=self.vosk_listen_loop, daemon=True)
            self.active_voice_thread.start()
        else:
            self.listening = False
            self.btn_voice.setText("Start Listening")
            self.btn_voice.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 10px;")
            self.bridge.voice_status_signal.emit("Idle")

    def vosk_listen_loop(self):
        q = queue.Queue()

        def callback(indata, frames, time, status):
            if status:
                pass
            q.put(bytes(indata))

        try:
            if not os.path.exists(self.model_path):
                self.bridge.log_signal.emit(f"Vosk model not found at {self.model_path}.")
                return

            device_info = sd.query_devices(None, 'input')
            samplerate = int(device_info['default_samplerate'])

            model = vosk.Model(self.model_path)
            recognizer = vosk.KaldiRecognizer(model, samplerate)

            self.bridge.log_signal.emit(f"Microphone active (Sample rate: {samplerate}). Say commands offline!")

            with sd.RawInputStream(samplerate=samplerate, blocksize=8000, dtype='int16',
                                   channels=1, callback=callback):
                while self.listening:
                    data = q.get()
                    if recognizer.AcceptWaveform(data):
                        result = json.loads(recognizer.Result())
                        text = result.get("text", "").lower()
                        if text:
                            self.bridge.log_signal.emit(f"Recognized (Offline): '{text}'")
                            self.process_voice_command(text)
                    else:
                        pass
        except Exception as e:
            self.bridge.log_signal.emit(f"Voice Recognition Error: {e}")
            self.listening = False
            self.bridge.voice_status_signal.emit("Error")

    def process_voice_command(self, text):
        # We need specific actions for time periods:
        # 1.0 second for forward, backward, up, down
        # 0.5 seconds for left, right (slide), turn left, turn right

        linear_x = 0.0
        linear_y = 0.0
        linear_z = 0.0
        angular_z = 0.0
        duration = 0.0
        command_matched = ""

        # Forward / Backward
        if any(w in text for w in ["forward", "front", "go"]):
            linear_x = 0.5
            duration = 1.0
            command_matched = "FORWARD"
        elif any(w in text for w in ["back", "backward"]):
            linear_x = -0.5
            duration = 1.0
            command_matched = "BACKWARD"

        # Up / Down
        elif any(w in text for w in ["up", "ascend", "climb"]):
            linear_z = 0.5
            duration = 1.0
            command_matched = "ASCEND"
        elif any(w in text for w in ["down", "descend", "land"]):
            linear_z = -0.5
            duration = 1.0
            command_matched = "DESCEND"

        # Yaw left/right or slide left/right
        elif "turn left" in text or "rotate left" in text:
            angular_z = 0.5
            duration = 0.5
            command_matched = "ROTATE LEFT"
        elif "turn right" in text or "rotate right" in text:
            angular_z = -0.5
            duration = 0.5
            command_matched = "ROTATE RIGHT"
        elif "left" in text:
            linear_y = 0.5
            duration = 0.5
            command_matched = "SLIDE LEFT"
        elif "right" in text:
            linear_y = -0.5
            duration = 0.5
            command_matched = "SLIDE RIGHT"

        # Stop
        elif any(w in text for w in ["stop", "hold", "hover"]):
            self.emergency_stop()
            return

        if command_matched:
            self.bridge.log_signal.emit(f"Executing: {command_matched}")
            self.ros_node.publish_twist(linear_x, linear_y, linear_z, angular_z)
            self.voice_stop_timer.start(int(duration * 1000))


def main():
    # Initialize PyQt5 Application first (critical for QObject instantiations like SignalBridge)
    app = QApplication(sys.argv)

    # Initialize ROS2
    rclpy.init(args=None)

    # Create SignalBridge
    signal_bridge = SignalBridge()

    # Create ROS2 Node with SignalBridge reference
    ros_node = ROS2Bridge(signal_bridge)

    # Spin ROS2 in a separate background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Create and show GUI
    gui = DroneGUI(ros_node, signal_bridge)
    gui.show()

    # Run PyQt5 Loop
    exit_code = app.exec_()

    # Clean up ROS2
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
