#!/usr/bin/env python3
#
# Robot Arm Control Application
# Copyright (C) 2026 BigHead theBit <student15.tigo.robotics@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
"""
Robot Arm Control Application
Unified application combining:
- Tkinter GUI for configuration and manual motor control
- WebSocket server for browser-based IK interface
- Motor control via single serial port connection
"""

import asyncio
import websockets
import json
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import os
from typing import Dict, Optional
import math

# Import matplotlib for visualization
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import numpy as np
    HAS_MATPLOTLIB = True
except ImportError:
    print("Warning: matplotlib not available. Visualization disabled.")
    HAS_MATPLOTLIB = False

# Import DM_CAN if available
try:
    from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type
    HAS_DM_CAN = True
except ImportError:
    print("Warning: DM_CAN not available. Motor control will be disabled.")
    HAS_DM_CAN = False


class RobotArmControlApp:
    """Unified application with GUI, WebSocket server, and motor control"""

    def __init__(self):
        # Configuration paths
        self.motor_config_file = "sensor_config/actor_config/config.json"
        self.robot_config_file = "robot_arm_config.json"

        # Motor configuration
        self.motor_config = {
            'serial_port': '/dev/ttyACM0',
            'baudrate': 921600,
            'motors': []
        }

        # Robot arm configuration
        self.config = {
            'dof': 6,
            'motorMapping': {},
            'gripperMotorId': '0x06',
            'gripperRangeMin': 0,
            'gripperRangeMax': 90,
            'jointLimits': [],
            'segmentLengths': [100, 100, 100, 50, 50, 30],
            'collisionEnabled': True,
            'minSegmentDistance': 20,
            'basePosition': {'x': 400, 'y': 550}
        }

        # Detected motor positions (populated by detect_current_position)
        self.detected_positions = {}

        # Motor control state
        self.serial_port = None
        self.motor_control = None
        self.motors = {}
        self.is_connected = False
        self.motor_enabled = {}

        # Safety limits for WebSocket commands
        self.max_position_jump = 2.0  # rad
        self.position_limit = 10.0    # rad
        self.last_positions = {}

        # WebSocket server
        self.websocket_server = None
        self.websocket_port = 8765
        self.connected_clients = set()

        # Load configurations
        self.load_motor_config()
        self.load_robot_config()

        # Create GUI
        self.create_gui()

    @staticmethod
    def to_json_serializable(value):
        """Convert numpy types to JSON-serializable Python types"""
        import numpy as np
        if isinstance(value, (np.float32, np.float64)):
            return float(value)
        elif isinstance(value, (np.int32, np.int64)):
            return int(value)
        elif isinstance(value, dict):
            return {k: RobotArmControlApp.to_json_serializable(v) for k, v in value.items()}
        elif isinstance(value, list):
            return [RobotArmControlApp.to_json_serializable(v) for v in value]
        return value

    def create_gui(self):
        """Create the Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("Robot Arm Control - Unified Application")
        self.root.geometry("950x950")

        # Create toolbar at top
        toolbar = ttk.Frame(self.root, relief=tk.RAISED, borderwidth=1)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        # Quick action buttons
        ttk.Button(toolbar, text="🌐 Open Web Interface",
                   command=self.open_web_interface,
                   style='Accent.TButton').pack(side=tk.LEFT, padx=5)

        ttk.Button(toolbar, text="🔌 Quick Connect",
                   command=self.quick_connect).pack(side=tk.LEFT, padx=5)

        ttk.Button(toolbar, text="⚙️ Motor Config",
                   command=lambda: self.notebook.select(3)).pack(side=tk.LEFT, padx=5)

        # Status indicator
        self.toolbar_status = ttk.Label(toolbar, text="● Not Connected", foreground="red")
        self.toolbar_status.pack(side=tk.RIGHT, padx=10)

        # Create notebook with tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Tab 1: Connection & All Motors Enable/Disable
        self.control_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.control_tab, text="1. Connection & Motors")
        self.setup_control_tab()

        # Tab 2: Robot Mapping (which motor does what)
        self.config_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.config_tab, text="2. Robot Mapping")
        self.setup_config_tab()

        # Tab 3: Motor Management (add/remove/edit motors)
        self.motor_mgmt_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.motor_mgmt_tab, text="3. Motor Management")
        self.setup_motor_management_tab()

        # Tab 4: WebSocket Server Status
        self.server_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.server_tab, text="4. Server Status")
        self.setup_server_tab()

        # Status bar
        self.status_bar = ttk.Label(self.root, text="Ready", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def setup_control_tab(self):
        """Setup connection and motor control tab"""
        # Serial Connection Frame
        conn_frame = ttk.LabelFrame(self.control_tab, text="Serial Connection", padding="10")
        conn_frame.pack(fill=tk.X, padx=10, pady=10)

        # Port selection
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X, pady=5)

        ttk.Label(port_frame, text="Serial Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=self.motor_config['serial_port'])
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=30)
        self.port_combo.pack(side=tk.LEFT, padx=10)

        ttk.Button(port_frame, text="Refresh Ports", command=self.refresh_ports).pack(side=tk.LEFT)

        # Baudrate
        baud_frame = ttk.Frame(conn_frame)
        baud_frame.pack(fill=tk.X, pady=5)

        ttk.Label(baud_frame, text="Baudrate:").pack(side=tk.LEFT)
        self.baudrate_var = tk.StringVar(value=str(self.motor_config['baudrate']))
        baudrate_combo = ttk.Combobox(baud_frame, textvariable=self.baudrate_var,
                                      values=['921600', '115200', '57600', '38400', '19200', '9600'],
                                      width=15)
        baudrate_combo.pack(side=tk.LEFT, padx=10)

        # Connection buttons
        button_frame = ttk.Frame(conn_frame)
        button_frame.pack(fill=tk.X, pady=10)

        self.connect_btn = ttk.Button(button_frame, text="Connect", command=self.connect_motors)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.disconnect_btn = ttk.Button(button_frame, text="Disconnect",
                                         command=self.disconnect_motors, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)

        # Connection status
        self.conn_status_label = ttk.Label(conn_frame, text="Status: Not Connected",
                                           foreground="red")
        self.conn_status_label.pack(pady=5)

        # Motor Control Frame
        motor_frame = ttk.LabelFrame(self.control_tab, text="All Motors (Joints + Gripper)", padding="10")
        motor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Info label
        info_label = ttk.Label(motor_frame,
                              text="Enable/disable individual motors. All motors from Tab 3 appear here.",
                              font=('Arial', 8, 'italic'))
        info_label.pack(pady=5)

        # Enable/Disable All buttons
        all_button_frame = ttk.Frame(motor_frame)
        all_button_frame.pack(fill=tk.X, pady=5)

        ttk.Button(all_button_frame, text="✓ Enable All Motors",
                   command=self.enable_all_motors).pack(side=tk.LEFT, padx=5)
        ttk.Button(all_button_frame, text="✗ Disable All Motors",
                   command=self.disable_all_motors).pack(side=tk.LEFT, padx=5)

        # Individual motor controls
        self.motor_controls_frame = ttk.Frame(motor_frame)
        self.motor_controls_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Populate initial ports
        self.refresh_ports()

    def setup_config_tab(self):
        """Setup robot configuration tab"""
        # Create scrollable frame
        canvas = tk.Canvas(self.config_tab)
        scrollbar = ttk.Scrollbar(self.config_tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        config_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Make scrollable frame expand to canvas width
        def configure_canvas_width(event):
            canvas.itemconfig(config_window, width=event.width)
        canvas.bind('<Configure>', configure_canvas_width)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # DOF Configuration
        dof_frame = ttk.LabelFrame(scrollable_frame, text="Robot Configuration", padding="10")
        dof_frame.pack(fill=tk.X, padx=10, pady=10)

        # DOF setting
        dof_row = ttk.Frame(dof_frame)
        dof_row.pack(fill=tk.X, pady=5)

        ttk.Label(dof_row, text="Number of joints (DOF):").pack(side=tk.LEFT)
        self.dof_var = tk.IntVar(value=self.config['dof'])
        dof_spin = ttk.Spinbox(dof_row, from_=1, to=8, width=10,
                               textvariable=self.dof_var, command=self.on_dof_changed)
        dof_spin.pack(side=tk.LEFT, padx=10)

        # Position detection
        pos_detect_frame = ttk.Frame(dof_frame)
        pos_detect_frame.pack(fill=tk.X, pady=10)

        ttk.Label(pos_detect_frame,
                 text="Before using the robot arm, detect its current position:",
                 font=('Arial', 9)).pack(anchor=tk.W, pady=(0, 5))

        # Gravity detection motor selector
        gravity_motor_row = ttk.Frame(pos_detect_frame)
        gravity_motor_row.pack(fill=tk.X, pady=5)

        ttk.Label(gravity_motor_row, text="Gravity reference motor:").pack(side=tk.LEFT)
        self.gravity_motor_var = tk.StringVar(value="Joint 1")
        self.gravity_motor_combo = ttk.Combobox(gravity_motor_row,
                                                textvariable=self.gravity_motor_var,
                                                width=15,
                                                state='readonly')
        self.gravity_motor_combo['values'] = ['Joint 1']  # Will be updated when motors configured
        self.gravity_motor_combo.pack(side=tk.LEFT, padx=10)

        ttk.Label(gravity_motor_row,
                 text="(Single motor used to detect gravity)",
                 font=('Arial', 8, 'italic'),
                 foreground='gray').pack(side=tk.LEFT, padx=5)

        ttk.Button(pos_detect_frame, text="🔍 Detect Current Position",
                  command=self.detect_current_position,
                  style='Accent.TButton').pack(anchor=tk.W, pady=5)

        self.position_status_label = ttk.Label(pos_detect_frame,
                                              text="Status: Not detected",
                                              font=('Arial', 9, 'italic'),
                                              foreground='gray')
        self.position_status_label.pack(anchor=tk.W, pady=(5, 0))

        # Info section
        info_frame = ttk.Frame(scrollable_frame)
        info_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(info_frame,
                 text="Assign which motor controls each joint and the gripper.\nMotors must be added in Tab 3 (Motor Management) first.",
                 font=('Arial', 9), justify=tk.LEFT).pack(anchor=tk.W)

        # Motor Mapping
        mapping_frame = ttk.LabelFrame(scrollable_frame, text="Motor Function Mapping", padding="10")
        mapping_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(mapping_frame, text="Assign motor IDs to joints and gripper:",
                 font=('Arial', 9, 'bold')).pack(anchor=tk.W, pady=(0, 5))

        # Create canvas with scrollbar - increased height for better visibility
        self.mapping_canvas = tk.Canvas(mapping_frame, height=350, bg='white')
        mapping_scrollbar = ttk.Scrollbar(mapping_frame, orient="vertical", command=self.mapping_canvas.yview)
        self.mapping_content = ttk.Frame(self.mapping_canvas)

        self.mapping_content.bind(
            "<Configure>",
            lambda e: self.mapping_canvas.configure(scrollregion=self.mapping_canvas.bbox("all"))
        )

        # Create window that expands to canvas width
        self.mapping_canvas_window = self.mapping_canvas.create_window((0, 0), window=self.mapping_content, anchor="nw")
        self.mapping_canvas.configure(yscrollcommand=mapping_scrollbar.set)

        # Bind canvas resize to update content width
        self.mapping_canvas.bind('<Configure>', lambda e: self.mapping_canvas.itemconfig(self.mapping_canvas_window, width=e.width))

        self.mapping_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        mapping_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.motor_entries = []
        self.motor_reverse_entries = []
        self.limit_min_entries = []
        self.limit_max_entries = []
        self.gripper_reverse_var = None

        self.populate_motor_mappings()

        # Collision Detection
        collision_frame = ttk.LabelFrame(scrollable_frame, text="Collision Detection", padding="10")
        collision_frame.pack(fill=tk.X, padx=10, pady=10)

        self.collision_var = tk.BooleanVar(value=self.config['collisionEnabled'])
        collision_check = ttk.Checkbutton(collision_frame, text="Enable collision detection",
                                         variable=self.collision_var)
        collision_check.pack(anchor=tk.W)

        distance_frame = ttk.Frame(collision_frame)
        distance_frame.pack(fill=tk.X, pady=5)

        ttk.Label(distance_frame, text="Minimum segment distance:").pack(side=tk.LEFT)
        self.distance_var = tk.IntVar(value=self.config['minSegmentDistance'])
        distance_spin = ttk.Spinbox(distance_frame, from_=5, to=50, width=10,
                                   textvariable=self.distance_var)
        distance_spin.pack(side=tk.LEFT, padx=10)
        ttk.Label(distance_frame, text="pixels").pack(side=tk.LEFT)

        # Action Buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(button_frame, text="Save Config & Apply",
                  command=self.save_and_apply_config,
                  style='Accent.TButton').pack(side=tk.LEFT, padx=5)

        ttk.Button(button_frame, text="Open Web Interface",
                  command=self.open_web_interface).pack(side=tk.LEFT, padx=5)

    def setup_server_tab(self):
        """Setup WebSocket server status tab"""
        # Server status
        status_frame = ttk.LabelFrame(self.server_tab, text="Server Status", padding="10")
        status_frame.pack(fill=tk.X, padx=10, pady=10)

        self.server_status_label = ttk.Label(status_frame, text="WebSocket Server: Starting...",
                                             font=('Arial', 10, 'bold'))
        self.server_status_label.pack(pady=5)

        self.server_url_label = ttk.Label(status_frame, text=f"URL: ws://localhost:{self.websocket_port}")
        self.server_url_label.pack(pady=5)

        # Connected clients
        clients_frame = ttk.LabelFrame(self.server_tab, text="Connected Clients", padding="10")
        clients_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.clients_text = tk.Text(clients_frame, height=10, state=tk.DISABLED)
        self.clients_text.pack(fill=tk.BOTH, expand=True)

        # Server log
        log_frame = ttk.LabelFrame(self.server_tab, text="Server Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.log_text = tk.Text(log_frame, height=15, state=tk.DISABLED)
        log_scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def setup_motor_management_tab(self):
        """Setup motor management tab for adding/removing motors"""
        # Info label
        info_frame = ttk.Frame(self.motor_mgmt_tab)
        info_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(info_frame, text="Manage motors in the motor configuration file",
                 font=('Arial', 10, 'bold')).pack(anchor=tk.W)
        ttk.Label(info_frame,
                 text="These motors are loaded from: sensor_config/actor_config/config.json",
                 font=('Arial', 8, 'italic')).pack(anchor=tk.W, pady=5)

        # Motor list
        list_frame = ttk.LabelFrame(self.motor_mgmt_tab, text="Current Motors", padding="10")
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Treeview for motor list
        columns = ('slave_id', 'master_id', 'motor_type', 'kp', 'kd')
        self.motor_tree = ttk.Treeview(list_frame, columns=columns, show='headings', height=10)

        self.motor_tree.heading('slave_id', text='Slave ID')
        self.motor_tree.heading('master_id', text='Master ID')
        self.motor_tree.heading('motor_type', text='Motor Type')
        self.motor_tree.heading('kp', text='Kp')
        self.motor_tree.heading('kd', text='Kd')

        self.motor_tree.column('slave_id', width=100)
        self.motor_tree.column('master_id', width=100)
        self.motor_tree.column('motor_type', width=120)
        self.motor_tree.column('kp', width=80)
        self.motor_tree.column('kd', width=80)

        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.motor_tree.yview)
        self.motor_tree.configure(yscroll=scrollbar.set)

        self.motor_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Populate motor list
        self.refresh_motor_list()

        # Add/Remove buttons
        button_frame = ttk.Frame(self.motor_mgmt_tab)
        button_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(button_frame, text="➕ Add New Motor",
                  command=self.add_motor_dialog).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="✏️ Edit Selected",
                  command=self.edit_motor_dialog).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🗑️ Remove Selected",
                  command=self.remove_motor).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🔄 Refresh",
                  command=self.refresh_motor_list).pack(side=tk.LEFT, padx=5)

    # Configuration Methods
    def load_motor_config(self):
        """Load motor configuration"""
        try:
            if os.path.exists(self.motor_config_file):
                with open(self.motor_config_file, 'r') as f:
                    data = json.load(f)
                    self.motor_config['serial_port'] = data.get('serial_port', '/dev/ttyACM0')
                    self.motor_config['baudrate'] = int(data.get('baudrate', 921600))
                    self.motor_config['motors'] = data.get('motors', [])
                    self.log(f"Loaded motor config: {len(self.motor_config['motors'])} motors")
        except Exception as e:
            self.log(f"Error loading motor config: {e}")

    def load_robot_config(self):
        """Load robot arm configuration"""
        try:
            if os.path.exists(self.robot_config_file):
                with open(self.robot_config_file, 'r') as f:
                    self.config = json.load(f)
                    self.log(f"Loaded robot config: {self.config['dof']} DOF")
        except Exception as e:
            self.log(f"Error loading robot config: {e}")

    def detect_gravity_direction(self, joint_angles_rad, reference_joint_idx=0):
        """
        Detect gravity direction from a single reference motor angle.
        This is more accurate than forward kinematics as it directly measures
        the motor's orientation relative to gravity.

        Args:
            joint_angles_rad: List of joint angles in radians
            reference_joint_idx: Index of joint to use as gravity reference (default: 0 = Joint 1)

        Returns:
            dict with direction, angle, and interpretation
        """
        import math

        if reference_joint_idx >= len(joint_angles_rad):
            reference_joint_idx = 0

        # Get the reference motor angle
        motor_angle_rad = joint_angles_rad[reference_joint_idx]
        motor_angle_deg = motor_angle_rad * 180 / math.pi

        # For a motor mounted on a vertical base:
        # 0° = horizontal right, 90° = vertical up, 180° = horizontal left, -90° = vertical down
        # The motor angle directly tells us which way the arm segment is pointing

        angle_from_horizontal = motor_angle_deg
        angle_from_vertical = angle_from_horizontal - 90  # Convert to vertical reference

        # Determine primary direction based on motor angle
        direction = ""
        interpretation = ""
        torque_info = ""

        if -45 <= angle_from_horizontal < 45:
            direction = "→ RIGHT"
            interpretation = "Motor is in HORIZONTAL position pointing RIGHT."
            torque_info = "Moderate torque needed to counteract gravity sag."
        elif 45 <= angle_from_horizontal < 135:
            direction = "↑ UPWARD"
            interpretation = "Motor is in VERTICAL position pointing UPWARD."
            torque_info = "HIGH torque required - motor fighting gravity."
        elif angle_from_horizontal >= 135 or angle_from_horizontal < -135:
            direction = "← LEFT"
            interpretation = "Motor is in HORIZONTAL position pointing LEFT."
            torque_info = "Moderate torque needed to counteract gravity sag."
        else:  # -135 <= angle_from_horizontal < -45
            direction = "↓ DOWNWARD"
            interpretation = "Motor is in VERTICAL position pointing DOWNWARD."
            torque_info = "MINIMAL torque - gravity assists position."

        # Additional context based on exact angle
        if abs(angle_from_vertical) < 10:
            interpretation += "\n⚠️ Nearly vertical - maximum gravity load!"
        elif abs(angle_from_horizontal) < 10:
            interpretation += "\n✓ Nearly horizontal - moderate gravity load."

        interpretation += f"\n\n{torque_info}"
        interpretation += f"\n\nReference: Joint {reference_joint_idx + 1} at {motor_angle_deg:.1f}°"

        return {
            'direction': direction,
            'motor_angle_deg': motor_angle_deg,
            'angle_from_horizontal': angle_from_horizontal,
            'angle_from_vertical': angle_from_vertical,
            'reference_joint': reference_joint_idx + 1,
            'interpretation': interpretation
        }

    def detect_current_position(self):
        """
        Safely detect current position of all configured motors.
        Uses minimal torque to read current positions without moving the arm.
        """
        if not self.is_connected:
            messagebox.showerror("Not Connected", "Please connect to serial port first (Tab 1)")
            return

        if not self.motors:
            messagebox.showerror("No Motors", "Please configure motors first (Tab 3)")
            return

        if not self.config.get('motorMapping'):
            messagebox.showerror("No Mapping", "Please configure motor mapping first (Tab 2)")
            return

        self.position_status_label.config(text="Status: Detecting...", foreground='orange')
        self.log("Starting position detection...")

        try:
            detected = {}
            motor_reverse_lookup = {}

            # Build reverse lookup
            for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
                is_reversed = self.config.get('motorReverse', {}).get(joint_idx_str, False)
                motor_reverse_lookup[motor_id] = is_reversed

            # Check gripper motor
            gripper_id = self.config.get('gripperMotorId', '')
            if gripper_id:
                motor_reverse_lookup[gripper_id] = self.config.get('gripperReverse', False)

            # Query each configured motor
            all_motor_ids = set(self.config.get('motorMapping', {}).values())
            if gripper_id:
                all_motor_ids.add(gripper_id)

            for motor_id in all_motor_ids:
                if motor_id not in self.motors:
                    self.log(f"Warning: Motor {motor_id} not found in configured motors")
                    continue

                motor = self.motors[motor_id]

                try:
                    # Enable motor with minimal torque (safe for position reading)
                    self.motor_control.enable(motor)

                    # Refresh to get current status
                    self.motor_control.refresh_motor_status(motor)

                    # Read current position
                    current_pos = float(motor.getPosition())

                    # Apply reversal if needed
                    actual_pos = current_pos
                    if motor_reverse_lookup.get(motor_id, False):
                        actual_pos = -current_pos
                        self.log(f"Motor {motor_id}: raw={current_pos:.3f} rad, reversed={actual_pos:.3f} rad")
                    else:
                        self.log(f"Motor {motor_id}: position={current_pos:.3f} rad ({current_pos * 180 / 3.14159:.1f}°)")

                    detected[motor_id] = actual_pos

                    # Store for safety checks
                    self.last_positions[motor_id] = current_pos

                except Exception as e:
                    self.log(f"Error reading motor {motor_id}: {e}")
                    messagebox.showerror("Detection Error", f"Failed to read motor {motor_id}: {e}")
                    self.position_status_label.config(text="Status: Detection failed", foreground='red')
                    return

            # Store detected positions
            self.detected_positions = detected

            # Build angle array for joints (excluding gripper)
            joint_angles_rad = []
            joint_angles_deg = []
            for i in range(self.config['dof']):
                motor_id = self.config.get('motorMapping', {}).get(str(i))
                if motor_id and motor_id in detected:
                    angle_rad = detected[motor_id]
                    angle_deg = angle_rad * 180 / 3.14159
                    joint_angles_rad.append(angle_rad)
                    joint_angles_deg.append(angle_deg)
                else:
                    joint_angles_rad.append(0.0)
                    joint_angles_deg.append(0.0)

            # Get selected reference joint for gravity detection
            gravity_motor_selection = self.gravity_motor_var.get()  # e.g., "Joint 1"
            reference_joint_idx = 0
            try:
                # Parse "Joint X" to get index (X-1)
                reference_joint_idx = int(gravity_motor_selection.split()[-1]) - 1
            except:
                reference_joint_idx = 0

            # Detect gravity direction from selected joint angle
            gravity_info = self.detect_gravity_direction(joint_angles_rad, reference_joint_idx)

            # Update status
            self.position_status_label.config(
                text=f"Status: ✓ Detected {len(detected)} motors | Gravity: {gravity_info['direction']}",
                foreground='green'
            )

            # Collect motor telemetry for visualization
            motor_telemetry = {}
            for motor_id, motor in self.motors.items():
                if motor_id in detected:
                    try:
                        self.motor_control.refresh_motor_status(motor)

                        # Try to get temperature, default to 25°C if not available
                        try:
                            temperature = motor.getTemperature()
                        except (AttributeError, Exception):
                            temperature = 25.0  # Default room temperature

                        motor_telemetry[motor_id] = {
                            'position': float(motor.getPosition()),
                            'velocity': float(motor.getVelocity()),
                            'torque': float(motor.getTorque()),
                            'temperature': float(temperature)
                        }
                    except Exception as e:
                        self.log(f"Error reading telemetry for {motor_id}: {e}")

            # Show visualization window
            self.show_gravity_detection_window(
                joint_angles_deg,
                detected,
                gravity_info,
                motor_telemetry,
                reference_joint_idx
            )

            # Broadcast detected positions and gravity info to web interface
            asyncio.run_coroutine_threadsafe(
                self.broadcast_detected_position(detected, gravity_info),
                self.loop
            )

            self.log(f"Position detection completed successfully - Gravity: {gravity_info['direction']}")

        except Exception as e:
            self.log(f"Position detection error: {e}")
            messagebox.showerror("Error", f"Position detection failed: {e}")
            self.position_status_label.config(text="Status: Error", foreground='red')

    def show_gravity_detection_window(self, joint_angles_deg, detected_positions,
                                       gravity_info, motor_telemetry, reference_joint_idx):
        """
        Show interactive visualization window with continuous monitoring capability
        """
        # Create new window
        viz_window = tk.Toplevel(self.root)
        viz_window.title("Gravity Detection Analysis - Interactive Monitor")
        viz_window.geometry("1400x900")

        # Detection state
        detection_state = {
            'running': False,
            'data': {
                'timestamps': [],
                'positions': {},
                'velocities': {},
                'torques': {},
                'temperatures': {}
            },
            'current_gravity': gravity_info,
            'current_telemetry': motor_telemetry
        }

        # Detection parameters
        params = {
            'duration': tk.DoubleVar(value=3.0),  # seconds
            'sample_rate': tk.IntVar(value=10),   # Hz
            'movement_amplitude': tk.DoubleVar(value=0.05),  # rad (micro-movement)
            'movement_speed': tk.DoubleVar(value=1.0),  # rad/s
            'position_threshold': tk.DoubleVar(value=0.1),  # rad
            'velocity_threshold': tk.DoubleVar(value=0.05),  # rad/s
            'torque_threshold': tk.DoubleVar(value=0.5)  # N⋅m
        }

        # Motor reverse settings (one checkbox per motor)
        motor_reverse_vars = {}

        # Create main frame
        main_frame = ttk.Frame(viz_window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Control Panel (left side)
        control_frame = ttk.LabelFrame(main_frame, text="Detection Controls", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # Detection parameters
        ttk.Label(control_frame, text="Detection Duration:", font=('Arial', 9, 'bold')).pack(anchor=tk.W, pady=(5, 2))
        duration_frame = ttk.Frame(control_frame)
        duration_frame.pack(fill=tk.X, pady=2)
        ttk.Scale(duration_frame, from_=0.5, to=10.0, variable=params['duration'],
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT)
        ttk.Label(duration_frame, textvariable=params['duration'],
                 font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        ttk.Label(duration_frame, text="sec", font=('Arial', 8)).pack(side=tk.LEFT)

        ttk.Label(control_frame, text="Sample Rate:", font=('Arial', 9, 'bold')).pack(anchor=tk.W, pady=(10, 2))
        rate_frame = ttk.Frame(control_frame)
        rate_frame.pack(fill=tk.X, pady=2)
        ttk.Scale(rate_frame, from_=1, to=50, variable=params['sample_rate'],
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT)
        ttk.Label(rate_frame, textvariable=params['sample_rate'],
                 font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        ttk.Label(rate_frame, text="Hz", font=('Arial', 8)).pack(side=tk.LEFT)

        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # Micro-movement parameters
        ttk.Label(control_frame, text="Micro-Movement (Gravity Test):", font=('Arial', 10, 'bold')).pack(anchor=tk.W, pady=(5, 5))

        ttk.Label(control_frame, text="Movement Amplitude:", font=('Arial', 9, 'bold')).pack(anchor=tk.W, pady=(5, 2))
        amplitude_frame = ttk.Frame(control_frame)
        amplitude_frame.pack(fill=tk.X, pady=2)
        ttk.Scale(amplitude_frame, from_=0.01, to=0.2, variable=params['movement_amplitude'],
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT)
        ttk.Label(amplitude_frame, textvariable=params['movement_amplitude'],
                 font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        ttk.Label(amplitude_frame, text="rad", font=('Arial', 8)).pack(side=tk.LEFT)
        ttk.Label(control_frame, text="(~3° for 0.05 rad)", font=('Arial', 7, 'italic'),
                 foreground='gray').pack(anchor=tk.W, padx=5)

        ttk.Label(control_frame, text="Movement Speed:", font=('Arial', 9, 'bold')).pack(anchor=tk.W, pady=(5, 2))
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=2)
        ttk.Scale(speed_frame, from_=0.1, to=5.0, variable=params['movement_speed'],
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT)
        ttk.Label(speed_frame, textvariable=params['movement_speed'],
                 font=('Arial', 9)).pack(side=tk.LEFT, padx=5)
        ttk.Label(speed_frame, text="rad/s", font=('Arial', 8)).pack(side=tk.LEFT)

        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # Thresholds
        ttk.Label(control_frame, text="Detection Thresholds:", font=('Arial', 10, 'bold')).pack(anchor=tk.W, pady=(5, 5))

        # Position threshold
        ttk.Label(control_frame, text="Position Δ (rad):", font=('Arial', 9)).pack(anchor=tk.W, pady=(5, 2))
        pos_thresh_frame = ttk.Frame(control_frame)
        pos_thresh_frame.pack(fill=tk.X, pady=2)
        ttk.Entry(pos_thresh_frame, textvariable=params['position_threshold'],
                 width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(pos_thresh_frame, text="rad", font=('Arial', 8)).pack(side=tk.LEFT)

        # Velocity threshold
        ttk.Label(control_frame, text="Velocity Δ (rad/s):", font=('Arial', 9)).pack(anchor=tk.W, pady=(5, 2))
        vel_thresh_frame = ttk.Frame(control_frame)
        vel_thresh_frame.pack(fill=tk.X, pady=2)
        ttk.Entry(vel_thresh_frame, textvariable=params['velocity_threshold'],
                 width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(vel_thresh_frame, text="rad/s", font=('Arial', 8)).pack(side=tk.LEFT)

        # Torque threshold
        ttk.Label(control_frame, text="Torque Δ (N⋅m):", font=('Arial', 9)).pack(anchor=tk.W, pady=(5, 2))
        torque_thresh_frame = ttk.Frame(control_frame)
        torque_thresh_frame.pack(fill=tk.X, pady=2)
        ttk.Entry(torque_thresh_frame, textvariable=params['torque_threshold'],
                 width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(torque_thresh_frame, text="N⋅m", font=('Arial', 8)).pack(side=tk.LEFT)

        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # Motor Reverse Configuration
        ttk.Label(control_frame, text="Motor Reverse Settings:", font=('Arial', 10, 'bold')).pack(anchor=tk.W, pady=(5, 5))

        # Create scrollable frame for motor checkboxes
        reverse_canvas = tk.Canvas(control_frame, height=120)
        reverse_scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=reverse_canvas.yview)
        reverse_frame = ttk.Frame(reverse_canvas)

        reverse_frame.bind("<Configure>", lambda e: reverse_canvas.configure(scrollregion=reverse_canvas.bbox("all")))
        reverse_canvas.create_window((0, 0), window=reverse_frame, anchor="nw")
        reverse_canvas.configure(yscrollcommand=reverse_scrollbar.set)

        reverse_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        reverse_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Map motor IDs to joint indices for display
        motor_to_joint = {}
        for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
            motor_to_joint[motor_id] = int(joint_idx_str)

        # Add checkbox for each detected motor
        for motor_id in sorted(detected_positions.keys(), key=lambda x: motor_to_joint.get(x, 999)):
            joint_idx = motor_to_joint.get(motor_id, -1)
            label = f"Joint {joint_idx+1}" if joint_idx >= 0 else motor_id

            # Get current reverse setting from config
            is_reversed = self.config.get('motorReverse', {}).get(str(joint_idx), False) if joint_idx >= 0 else False
            motor_reverse_vars[motor_id] = tk.BooleanVar(value=is_reversed)

            ttk.Checkbutton(reverse_frame, text=f"{label} ({motor_id})",
                          variable=motor_reverse_vars[motor_id]).pack(anchor=tk.W, pady=2)

        # Save reverse settings button
        def save_reverse_settings():
            """Save reverse settings to config"""
            for motor_id, var in motor_reverse_vars.items():
                # Find joint index for this motor
                for joint_idx_str, mid in self.config.get('motorMapping', {}).items():
                    if mid == motor_id:
                        if var.get():
                            if 'motorReverse' not in self.config:
                                self.config['motorReverse'] = {}
                            self.config['motorReverse'][joint_idx_str] = True
                        else:
                            if 'motorReverse' in self.config and joint_idx_str in self.config['motorReverse']:
                                del self.config['motorReverse'][joint_idx_str]
            self.log("Motor reverse settings updated")
            status_text.set("Reverse settings saved!")
            viz_window.after(2000, lambda: status_text.set("Ready"))

        ttk.Button(control_frame, text="💾 Save Reverse Settings",
                  command=save_reverse_settings).pack(pady=5, fill=tk.X)

        ttk.Separator(control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)

        # Detection status
        status_text = tk.StringVar(value="Ready")
        status_label = ttk.Label(control_frame, textvariable=status_text,
                                font=('Arial', 10), foreground='blue')
        status_label.pack(pady=10)

        # Progress bar
        progress = ttk.Progressbar(control_frame, length=200, mode='determinate')
        progress.pack(pady=5)

        # Right side - Summary and plots
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Header with summary
        header_frame = ttk.LabelFrame(right_frame, text="Current Detection Summary", padding=10)
        header_frame.pack(fill=tk.X, pady=(0, 10))

        # Gravity direction (large and prominent)
        direction_label = ttk.Label(header_frame,
                                    text=f"🌍 GRAVITY: {gravity_info['direction']}",
                                    font=('Arial', 18, 'bold'),
                                    foreground='#667eea')
        direction_label.pack(pady=5)

        # Reference motor info
        ref_info = ttk.Label(header_frame,
                            text=f"Reference: Joint {reference_joint_idx + 1} | "
                                 f"Angle: {gravity_info['motor_angle_deg']:.1f}° | "
                                 f"From Vertical: {gravity_info['angle_from_vertical']:.1f}°",
                            font=('Arial', 11))
        ref_info.pack(pady=2)

        # Interpretation
        interp_label = ttk.Label(header_frame,
                                text=gravity_info['interpretation'],
                                font=('Arial', 9),
                                foreground='#666',
                                justify=tk.LEFT)
        interp_label.pack(pady=5)

        # Detection logic
        def run_continuous_detection():
            """Run continuous detection for specified duration"""
            if detection_state['running']:
                return  # Already running

            detection_state['running'] = True
            status_text.set("Detecting...")
            detect_button.config(state='disabled', text="⏱️ Detecting...")
            progress['value'] = 0

            # Clear previous data
            detection_state['data'] = {
                'timestamps': [],
                'positions': {},
                'velocities': {},
                'torques': {},
                'temperatures': {}
            }

            duration = params['duration'].get()
            sample_rate = params['sample_rate'].get()
            interval = 1.0 / sample_rate
            total_samples = int(duration * sample_rate)

            # Get micro-movement parameters
            amplitude = params['movement_amplitude'].get()
            speed = params['movement_speed'].get()

            # Get the reference motor ID (the one selected for gravity detection)
            reference_motor_id = None
            motor_to_joint = {}
            for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
                motor_to_joint[motor_id] = int(joint_idx_str)
                if int(joint_idx_str) == reference_joint_idx:
                    reference_motor_id = motor_id
                    break

            if not reference_motor_id:
                messagebox.showerror("Error", "Reference motor not found")
                detection_state['running'] = False
                return

            # Store initial position for the reference motor only
            if reference_motor_id in self.motors:
                motor = self.motors[reference_motor_id]
                self.motor_control.refresh_motor_status(motor)
                initial_position = motor.getPosition()
            else:
                messagebox.showerror("Error", f"Reference motor {reference_motor_id} not configured")
                detection_state['running'] = False
                return

            self.log(f"Starting micro-movement detection on Joint {reference_joint_idx+1} ({reference_motor_id}): amplitude={amplitude:.3f}rad, speed={speed:.3f}rad/s")

            def sample_motors(sample_num):
                if not detection_state['running'] or sample_num >= total_samples:
                    # Finish detection
                    detection_state['running'] = False
                    status_text.set("Detection Complete!")
                    detect_button.config(state='normal', text="🔍 Start Detection")
                    progress['value'] = 100
                    self.log(f"Detection complete: {len(detection_state['data']['timestamps'])} samples")

                    # Update plots with collected data
                    update_plots_with_data()
                    return

                try:
                    # Update progress
                    progress['value'] = (sample_num / total_samples) * 100
                    status_text.set(f"Sampling {sample_num+1}/{total_samples}")

                    # Calculate timestamp
                    timestamp = sample_num * interval

                    # Calculate micro-movement target (sinusoidal pattern)
                    # Use sine wave for smooth back-and-forth motion
                    import math
                    phase = (timestamp * 2 * math.pi) / duration  # Complete one cycle over duration
                    movement_offset = amplitude * math.sin(phase)

                    # Send movement command to ONLY the reference motor
                    if reference_motor_id in self.motors:
                        motor = self.motors[reference_motor_id]
                        target_position = initial_position + movement_offset

                        # Apply motor reversal if configured
                        is_reversed = motor_reverse_vars.get(reference_motor_id, tk.BooleanVar(value=False)).get()
                        if is_reversed:
                            target_position = initial_position - movement_offset

                        # Send position command with specified speed
                        self.motor_control.control_Pos_Vel(motor, target_position, speed)

                    # Wait a bit for motors to react, then sample
                    def do_sample():
                        # Sample all motors
                        timestamp = sample_num * interval
                        detection_state['data']['timestamps'].append(timestamp)

                        for motor_id, motor in self.motors.items():
                            if motor_id in detected_positions:
                                self.motor_control.refresh_motor_status(motor)

                                if motor_id not in detection_state['data']['positions']:
                                    detection_state['data']['positions'][motor_id] = []
                                    detection_state['data']['velocities'][motor_id] = []
                                    detection_state['data']['torques'][motor_id] = []
                                    detection_state['data']['temperatures'][motor_id] = []

                                # Try to get temperature, default to 25°C if not available
                                try:
                                    temperature = motor.getTemperature()
                                except (AttributeError, Exception):
                                    temperature = 25.0

                                detection_state['data']['positions'][motor_id].append(float(motor.getPosition()))
                                detection_state['data']['velocities'][motor_id].append(float(motor.getVelocity()))
                                detection_state['data']['torques'][motor_id].append(float(motor.getTorque()))
                                detection_state['data']['temperatures'][motor_id].append(float(temperature))

                        # Schedule next sample
                        viz_window.after(int(interval * 1000), lambda: sample_motors(sample_num + 1))

                    # Small delay to let motor start moving
                    viz_window.after(20, do_sample)

                except Exception as e:
                    self.log(f"Error during detection sampling: {e}")
                    detection_state['running'] = False
                    status_text.set("Error during detection")
                    detect_button.config(state='normal', text="🔍 Start Detection")

            # Start sampling
            sample_motors(0)

        def update_plots_with_data():
            """Update matplotlib plots with collected time-series data"""
            if not HAS_MATPLOTLIB or not detection_state['data']['timestamps']:
                return

            # Clear and recreate plots with time-series data
            for ax in [ax1, ax2, ax3, ax4]:
                ax.clear()

            timestamps = detection_state['data']['timestamps']

            # Map motor IDs to joint indices
            motor_to_joint = {}
            for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
                motor_to_joint[motor_id] = int(joint_idx_str)

            # Plot 1: Position over time
            ax1.set_ylabel('Position (rad)', fontsize=11, fontweight='bold')
            ax1.set_xlabel('Time (s)', fontsize=10)
            ax1.set_title('Motor Positions Over Time', fontsize=12, fontweight='bold')
            for motor_id in sorted(detection_state['data']['positions'].keys(),
                                  key=lambda x: motor_to_joint.get(x, 999)):
                joint_idx = motor_to_joint.get(motor_id, -1)
                label = f"J{joint_idx+1}" if joint_idx >= 0 else motor_id
                positions = detection_state['data']['positions'][motor_id]
                ax1.plot(timestamps, positions, label=label, marker='o' if len(timestamps) < 20 else '')
            ax1.legend()
            ax1.grid(alpha=0.3)

            # Plot 2: Velocity over time
            ax2.set_ylabel('Velocity (rad/s)', fontsize=11, fontweight='bold')
            ax2.set_xlabel('Time (s)', fontsize=10)
            ax2.set_title('Motor Velocities Over Time', fontsize=12, fontweight='bold')
            for motor_id in sorted(detection_state['data']['velocities'].keys(),
                                  key=lambda x: motor_to_joint.get(x, 999)):
                joint_idx = motor_to_joint.get(motor_id, -1)
                label = f"J{joint_idx+1}" if joint_idx >= 0 else motor_id
                velocities = detection_state['data']['velocities'][motor_id]
                ax2.plot(timestamps, velocities, label=label, marker='o' if len(timestamps) < 20 else '')
            ax2.legend()
            ax2.grid(alpha=0.3)
            ax2.axhline(y=params['velocity_threshold'].get(), color='red', linestyle='--', alpha=0.5, label='Threshold')
            ax2.axhline(y=-params['velocity_threshold'].get(), color='red', linestyle='--', alpha=0.5)

            # Plot 3: Torque over time
            ax3.set_ylabel('Torque (N⋅m)', fontsize=11, fontweight='bold')
            ax3.set_xlabel('Time (s)', fontsize=10)
            ax3.set_title('Motor Torques Over Time', fontsize=12, fontweight='bold')
            for motor_id in sorted(detection_state['data']['torques'].keys(),
                                  key=lambda x: motor_to_joint.get(x, 999)):
                joint_idx = motor_to_joint.get(motor_id, -1)
                label = f"J{joint_idx+1}" if joint_idx >= 0 else motor_id
                torques = detection_state['data']['torques'][motor_id]
                ax3.plot(timestamps, torques, label=label, marker='o' if len(timestamps) < 20 else '')
            ax3.legend()
            ax3.grid(alpha=0.3)

            # Plot 4: Temperature over time
            ax4.set_ylabel('Temperature (°C)', fontsize=11, fontweight='bold')
            ax4.set_xlabel('Time (s)', fontsize=10)
            ax4.set_title('Motor Temperatures Over Time', fontsize=12, fontweight='bold')
            for motor_id in sorted(detection_state['data']['temperatures'].keys(),
                                  key=lambda x: motor_to_joint.get(x, 999)):
                joint_idx = motor_to_joint.get(motor_id, -1)
                label = f"J{joint_idx+1}" if joint_idx >= 0 else motor_id
                temps = detection_state['data']['temperatures'][motor_id]
                ax4.plot(timestamps, temps, label=label, marker='o' if len(timestamps) < 20 else '')
            ax4.legend()
            ax4.grid(alpha=0.3)
            ax4.axhline(y=60, color='orange', linestyle='--', alpha=0.5, label='Warm')
            ax4.axhline(y=80, color='red', linestyle='--', alpha=0.5, label='Hot')

            canvas.draw()

        # Detection button (add after defining the function)
        detect_button = ttk.Button(control_frame, text="🔍 Start Detection",
                                   command=run_continuous_detection,
                                   style='Accent.TButton')
        detect_button.pack(pady=10, fill=tk.X)

        ttk.Button(control_frame, text="Close Window",
                  command=viz_window.destroy).pack(pady=5, fill=tk.X)

        if HAS_MATPLOTLIB:
            # Create matplotlib figure with subplots
            fig = Figure(figsize=(10, 8), dpi=100)

            # Map motor IDs to joint indices for display
            motor_to_joint = {}
            for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
                motor_to_joint[motor_id] = int(joint_idx_str)

            # Prepare data for all detected motors
            motor_ids = []
            joint_indices = []
            positions = []
            velocities = []
            torques = []
            temps = []

            for motor_id in sorted(detected_positions.keys(),
                                  key=lambda x: motor_to_joint.get(x, 999)):
                if motor_id in motor_telemetry:
                    motor_ids.append(motor_id)
                    joint_idx = motor_to_joint.get(motor_id, -1)
                    joint_indices.append(f"J{joint_idx+1}" if joint_idx >= 0 else motor_id)

                    telemetry = motor_telemetry[motor_id]
                    positions.append(telemetry['position'] * 180 / 3.14159)  # Convert to degrees
                    velocities.append(telemetry['velocity'])
                    torques.append(telemetry['torque'])
                    temps.append(telemetry['temperature'])

            # Subplot 1: Position (Bar chart)
            ax1 = fig.add_subplot(2, 2, 1)
            bars1 = ax1.bar(joint_indices, positions, color=['#ff6b6b' if i == reference_joint_idx else '#4ecdc4'
                                                              for i in range(len(positions))])
            ax1.set_ylabel('Position (degrees)', fontsize=11, fontweight='bold')
            ax1.set_title('Motor Positions', fontsize=12, fontweight='bold')
            ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            ax1.axhline(y=90, color='orange', linestyle='--', alpha=0.3, label='Vertical')
            ax1.axhline(y=-90, color='orange', linestyle='--', alpha=0.3)
            ax1.grid(axis='y', alpha=0.3)
            ax1.legend()

            # Add value labels on bars
            for i, (bar, val) in enumerate(zip(bars1, positions)):
                height = bar.get_height()
                ax1.text(bar.get_x() + bar.get_width()/2., height,
                        f'{val:.1f}°', ha='center', va='bottom' if height >= 0 else 'top',
                        fontsize=9, fontweight='bold')

            # Subplot 2: Velocity (Bar chart)
            ax2 = fig.add_subplot(2, 2, 2)
            bars2 = ax2.bar(joint_indices, velocities, color=['#ff6b6b' if i == reference_joint_idx else '#667eea'
                                                               for i in range(len(velocities))])
            ax2.set_ylabel('Velocity (rad/s)', fontsize=11, fontweight='bold')
            ax2.set_title('Motor Velocities', fontsize=12, fontweight='bold')
            ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            ax2.grid(axis='y', alpha=0.3)

            # Add value labels
            for bar, val in zip(bars2, velocities):
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height,
                        f'{val:.2f}', ha='center', va='bottom' if height >= 0 else 'top',
                        fontsize=9)

            # Subplot 3: Torque (Bar chart with gravity indication)
            ax3 = fig.add_subplot(2, 2, 3)
            bars3 = ax3.bar(joint_indices, torques, color=['#ff6b6b' if i == reference_joint_idx else '#ffd93d'
                                                            for i in range(len(torques))])
            ax3.set_ylabel('Torque (N⋅m)', fontsize=11, fontweight='bold')
            ax3.set_title('Motor Torques (Gravity Load)', fontsize=12, fontweight='bold')
            ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
            ax3.grid(axis='y', alpha=0.3)

            # Add value labels
            for bar, val in zip(bars3, torques):
                height = bar.get_height()
                ax3.text(bar.get_x() + bar.get_width()/2., height,
                        f'{val:.2f}', ha='center', va='bottom' if height >= 0 else 'top',
                        fontsize=9)

            # Subplot 4: Temperature (Bar chart)
            ax4 = fig.add_subplot(2, 2, 4)
            bars4 = ax4.bar(joint_indices, temps, color=['#ff6b6b' if i == reference_joint_idx else '#95e1d3'
                                                          for i in range(len(temps))])
            ax4.set_ylabel('Temperature (°C)', fontsize=11, fontweight='bold')
            ax4.set_title('Motor Temperatures', fontsize=12, fontweight='bold')
            ax4.axhline(y=60, color='orange', linestyle='--', alpha=0.5, label='Warm')
            ax4.axhline(y=80, color='red', linestyle='--', alpha=0.5, label='Hot')
            ax4.grid(axis='y', alpha=0.3)
            ax4.legend()

            # Add value labels
            for bar, val in zip(bars4, temps):
                height = bar.get_height()
                ax4.text(bar.get_x() + bar.get_width()/2., height,
                        f'{val:.1f}°', ha='center', va='bottom',
                        fontsize=9)

            fig.tight_layout()

            # Embed matplotlib figure in tkinter
            canvas_frame = ttk.Frame(right_frame)
            canvas_frame.pack(fill=tk.BOTH, expand=True)

            canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        else:
            # Fallback if matplotlib not available
            fallback_label = ttk.Label(main_frame,
                                      text="Matplotlib not installed.\nInstall with: pip install matplotlib numpy",
                                      font=('Arial', 12),
                                      foreground='red')
            fallback_label.pack(pady=20)

            # Show text summary instead
            text_frame = ttk.Frame(main_frame)
            text_frame.pack(fill=tk.BOTH, expand=True, pady=10)

            text_widget = tk.Text(text_frame, wrap=tk.WORD, font=('Courier', 10))
            text_widget.pack(fill=tk.BOTH, expand=True)

            summary = "DETECTED POSITIONS:\n\n"
            for i, angle_deg in enumerate(joint_angles_deg):
                summary += f"Joint {i+1}: {angle_deg:.1f}°\n"

            if motor_telemetry:
                summary += "\nMOTOR TELEMETRY:\n\n"
                for motor_id, telem in motor_telemetry.items():
                    summary += f"{motor_id}:\n"
                    summary += f"  Position: {telem['position']*180/3.14159:.1f}°\n"
                    summary += f"  Velocity: {telem['velocity']:.3f} rad/s\n"
                    summary += f"  Torque: {telem['torque']:.3f} N⋅m\n"
                    summary += f"  Temp: {telem['temperature']:.1f}°C\n\n"

            text_widget.insert('1.0', summary)
            text_widget.config(state='disabled')

        # Close button
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="Close",
                  command=viz_window.destroy,
                  style='Accent.TButton').pack()

    async def broadcast_detected_position(self, detected_positions, gravity_info):
        """Broadcast detected positions and gravity info to all connected WebSocket clients"""
        if not self.connected_clients:
            return

        message = {
            'command': 'position_detected',
            'positions': detected_positions,
            'gravity': {
                'direction': gravity_info['direction'],
                'angle_from_horizontal': gravity_info['angle_from_horizontal'],
                'angle_from_vertical': gravity_info['angle_from_vertical'],
                'reference_joint': gravity_info['reference_joint'],
                'motor_angle_deg': gravity_info['motor_angle_deg'],
                'interpretation': gravity_info['interpretation']
            }
        }

        # Send to all connected clients
        for client in list(self.connected_clients):
            try:
                await client.send(json.dumps(message))
            except:
                pass

    def save_and_apply_config(self):
        """Save robot configuration and send to browser"""
        try:
            # Update config from GUI
            self.config['dof'] = self.dof_var.get()
            self.config['collisionEnabled'] = self.collision_var.get()
            self.config['minSegmentDistance'] = self.distance_var.get()
            self.config['gripperMotorId'] = self.gripper_motor_var.get()

            # Update gripper reverse
            if self.gripper_reverse_var:
                self.config['gripperReverse'] = self.gripper_reverse_var.get()

            # Update gripper range
            if hasattr(self, 'gripper_range_min_var'):
                self.config['gripperRangeMin'] = self.gripper_range_min_var.get()
            if hasattr(self, 'gripper_range_max_var'):
                self.config['gripperRangeMax'] = self.gripper_range_max_var.get()

            # Update motor mapping
            self.config['motorMapping'] = {}
            for i, motor_var in enumerate(self.motor_entries):
                motor_id = motor_var.get()
                if motor_id:
                    self.config['motorMapping'][str(i)] = motor_id

            # Update motor reverse settings
            self.config['motorReverse'] = {}
            for i, reverse_var in enumerate(self.motor_reverse_entries):
                if reverse_var.get():  # Only save if reversed
                    self.config['motorReverse'][str(i)] = True

            # Update joint limits
            self.config['jointLimits'] = []
            for i in range(len(self.limit_min_entries)):
                self.config['jointLimits'].append({
                    'min': self.limit_min_entries[i].get(),
                    'max': self.limit_max_entries[i].get()
                })

            # Save to file
            with open(self.robot_config_file, 'w') as f:
                json.dump(self.config, f, indent=2)

            self.log("Configuration saved and applied")
            messagebox.showinfo("Success", "Configuration saved successfully!")

            # Broadcast to all connected WebSocket clients
            asyncio.run_coroutine_threadsafe(
                self.broadcast_config(),
                self.loop
            )

        except Exception as e:
            self.log(f"Error saving config: {e}")
            messagebox.showerror("Error", f"Failed to save configuration: {e}")

    def populate_motor_mappings(self):
        """Populate motor mapping entries based on DOF"""
        for widget in self.mapping_content.winfo_children():
            widget.destroy()

        self.motor_entries = []
        self.motor_reverse_entries = []
        self.limit_min_entries = []
        self.limit_max_entries = []

        dof = self.dof_var.get()

        # Update gravity motor selector dropdown
        gravity_options = [f"Joint {i+1}" for i in range(dof)]
        self.gravity_motor_combo['values'] = gravity_options
        if self.gravity_motor_var.get() not in gravity_options:
            self.gravity_motor_var.set(gravity_options[0] if gravity_options else "Joint 1")

        # Ensure we have enough joint limits
        while len(self.config['jointLimits']) < dof:
            self.config['jointLimits'].append({'min': -135, 'max': 135})

        # Header
        header_frame = ttk.Frame(self.mapping_content)
        header_frame.pack(fill=tk.X, pady=5, padx=5)

        ttk.Label(header_frame, text="Function", width=12, font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=3)
        ttk.Label(header_frame, text="Motor ID", width=18, font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=3)
        ttk.Label(header_frame, text="Reverse", width=8, font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=3)
        ttk.Label(header_frame, text="Min Angle", width=12, font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=3)
        ttk.Label(header_frame, text="Max Angle", width=12, font=('Arial', 9, 'bold')).pack(side=tk.LEFT, padx=3)

        # Available motor IDs from config
        available_motors = [m['slave_id'] for m in self.motor_config['motors']]

        # Create entry for each joint
        for i in range(dof):
            joint_frame = ttk.Frame(self.mapping_content)
            joint_frame.pack(fill=tk.X, pady=2, padx=5)

            # Joint label
            ttk.Label(joint_frame, text=f"Joint {i+1}", width=12,
                     font=('Arial', 9)).pack(side=tk.LEFT, padx=3)

            # Motor ID dropdown (from available motors)
            motor_id = self.config['motorMapping'].get(str(i), '')
            motor_var = tk.StringVar(value=motor_id)
            motor_combo = ttk.Combobox(joint_frame, textvariable=motor_var, width=16,
                                      values=[''] + available_motors)
            motor_combo.pack(side=tk.LEFT, padx=3)
            self.motor_entries.append(motor_var)

            # Reverse checkbox
            motor_reverse = self.config.get('motorReverse', {}).get(str(i), False)
            reverse_var = tk.BooleanVar(value=motor_reverse)
            reverse_check = ttk.Checkbutton(joint_frame, variable=reverse_var)
            reverse_check.pack(side=tk.LEFT, padx=12)
            self.motor_reverse_entries.append(reverse_var)

            # Min angle
            min_val = self.config['jointLimits'][i]['min']
            min_var = tk.IntVar(value=min_val)
            min_spin = ttk.Spinbox(joint_frame, from_=-180, to=180, width=10,
                                  textvariable=min_var)
            min_spin.pack(side=tk.LEFT, padx=3)
            self.limit_min_entries.append(min_var)

            # Max angle
            max_val = self.config['jointLimits'][i]['max']
            max_var = tk.IntVar(value=max_val)
            max_spin = ttk.Spinbox(joint_frame, from_=-180, to=180, width=10,
                                  textvariable=max_var)
            max_spin.pack(side=tk.LEFT, padx=3)
            self.limit_max_entries.append(max_var)

        # Add separator
        separator = ttk.Separator(self.mapping_content, orient='horizontal')
        separator.pack(fill=tk.X, pady=10, padx=5)

        # Gripper/End Effector row
        gripper_frame = ttk.Frame(self.mapping_content)
        gripper_frame.pack(fill=tk.X, pady=2, padx=5)

        ttk.Label(gripper_frame, text="Gripper", width=12,
                 font=('Arial', 9, 'bold'), foreground='blue').pack(side=tk.LEFT, padx=3)

        # Gripper motor ID
        gripper_motor_id = self.config.get('gripperMotorId', '0x06')
        self.gripper_motor_var = tk.StringVar(value=gripper_motor_id)
        gripper_combo = ttk.Combobox(gripper_frame, textvariable=self.gripper_motor_var,
                                     width=16, values=[''] + available_motors)
        gripper_combo.pack(side=tk.LEFT, padx=3)

        # Gripper reverse checkbox
        gripper_reverse = self.config.get('gripperReverse', False)
        self.gripper_reverse_var = tk.BooleanVar(value=gripper_reverse)
        gripper_reverse_check = ttk.Checkbutton(gripper_frame, variable=self.gripper_reverse_var)
        gripper_reverse_check.pack(side=tk.LEFT, padx=12)

        # Gripper range: Min
        gripper_min = self.config.get('gripperRangeMin', 0)
        self.gripper_range_min_var = tk.IntVar(value=gripper_min)
        ttk.Entry(gripper_frame, textvariable=self.gripper_range_min_var, width=8).pack(side=tk.LEFT, padx=3)

        # Gripper range: Max
        gripper_max = self.config.get('gripperRangeMax', 90)
        self.gripper_range_max_var = tk.IntVar(value=gripper_max)
        ttk.Entry(gripper_frame, textvariable=self.gripper_range_max_var, width=8).pack(side=tk.LEFT, padx=3)

        ttk.Label(gripper_frame, text="(Range: Min-Max degrees)", font=('Arial', 8, 'italic')).pack(side=tk.LEFT, padx=3)

    def on_dof_changed(self):
        """Handle DOF change"""
        self.populate_motor_mappings()

    def open_web_interface(self):
        """Open web interface in browser"""
        import webbrowser
        html_file = os.path.join(os.getcwd(), "RobotArmControl.html")
        webbrowser.open(f"file://{html_file}")
        self.log("Opened web interface in browser")

    # Motor Management Methods
    def refresh_motor_list(self):
        """Refresh the motor list in the treeview"""
        # Clear existing items
        for item in self.motor_tree.get_children():
            self.motor_tree.delete(item)

        # Add motors from config
        for motor in self.motor_config['motors']:
            self.motor_tree.insert('', 'end', values=(
                motor.get('slave_id', ''),
                motor.get('master_id', ''),
                motor.get('motor_type', ''),
                motor.get('kp', ''),
                motor.get('kd', '')
            ))

    def add_motor_dialog(self):
        """Open dialog to add new motor"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Add New Motor")
        dialog.geometry("400x350")
        dialog.transient(self.root)
        dialog.grab_set()

        # Motor fields
        fields = {}

        ttk.Label(dialog, text="Slave ID (e.g., 0x01):").grid(row=0, column=0, padx=10, pady=5, sticky='e')
        fields['slave_id'] = ttk.Entry(dialog, width=20)
        fields['slave_id'].grid(row=0, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Master ID (e.g., 0x11):").grid(row=1, column=0, padx=10, pady=5, sticky='e')
        fields['master_id'] = ttk.Entry(dialog, width=20)
        fields['master_id'].grid(row=1, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Motor Type:").grid(row=2, column=0, padx=10, pady=5, sticky='e')
        fields['motor_type'] = ttk.Combobox(dialog, width=18, values=[
            'DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V',
            'DM6006', 'DM8006', 'DM8009', 'DM10010', 'DM10010L',
            'DMG6220', 'DMH3510', 'DMH6215'
        ])
        fields['motor_type'].set('DM4340')
        fields['motor_type'].grid(row=2, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Kp:").grid(row=3, column=0, padx=10, pady=5, sticky='e')
        fields['kp'] = ttk.Entry(dialog, width=20)
        fields['kp'].insert(0, '1.0')
        fields['kp'].grid(row=3, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Kd:").grid(row=4, column=0, padx=10, pady=5, sticky='e')
        fields['kd'] = ttk.Entry(dialog, width=20)
        fields['kd'].insert(0, '0.1')
        fields['kd'].grid(row=4, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Speed:").grid(row=5, column=0, padx=10, pady=5, sticky='e')
        fields['speed'] = ttk.Entry(dialog, width=20)
        fields['speed'].insert(0, '1.0')
        fields['speed'].grid(row=5, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Torque Threshold:").grid(row=6, column=0, padx=10, pady=5, sticky='e')
        fields['torque_threshold'] = ttk.Entry(dialog, width=20)
        fields['torque_threshold'].insert(0, '1.0')
        fields['torque_threshold'].grid(row=6, column=1, padx=10, pady=5)

        def save_motor():
            new_motor = {
                'motor_type': fields['motor_type'].get(),
                'slave_id': fields['slave_id'].get(),
                'master_id': fields['master_id'].get(),
                'kp': fields['kp'].get(),
                'kd': fields['kd'].get(),
                'speed': fields['speed'].get(),
                'torque_threshold': fields['torque_threshold'].get()
            }
            self.motor_config['motors'].append(new_motor)
            self.save_motor_config()
            self.refresh_motor_list()
            dialog.destroy()

        button_frame = ttk.Frame(dialog)
        button_frame.grid(row=7, column=0, columnspan=2, pady=20)

        ttk.Button(button_frame, text="Save", command=save_motor).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=5)

    def edit_motor_dialog(self):
        """Open dialog to edit selected motor"""
        selection = self.motor_tree.selection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a motor to edit")
            return

        # Get selected motor data
        item = selection[0]
        values = self.motor_tree.item(item)['values']
        slave_id = values[0]

        # Find motor in config
        motor_idx = None
        motor_data = None
        for i, motor in enumerate(self.motor_config['motors']):
            if motor['slave_id'] == slave_id:
                motor_idx = i
                motor_data = motor
                break

        if motor_idx is None or motor_data is None:
            return

        # Create edit dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Edit Motor")
        dialog.geometry("400x350")
        dialog.transient(self.root)
        dialog.grab_set()

        # Motor fields pre-filled with current values
        fields = {}

        ttk.Label(dialog, text="Slave ID (e.g., 0x01):").grid(row=0, column=0, padx=10, pady=5, sticky='e')
        fields['slave_id'] = ttk.Entry(dialog, width=20)
        fields['slave_id'].insert(0, motor_data.get('slave_id', ''))
        fields['slave_id'].grid(row=0, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Master ID (e.g., 0x11):").grid(row=1, column=0, padx=10, pady=5, sticky='e')
        fields['master_id'] = ttk.Entry(dialog, width=20)
        fields['master_id'].insert(0, motor_data.get('master_id', ''))
        fields['master_id'].grid(row=1, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Motor Type:").grid(row=2, column=0, padx=10, pady=5, sticky='e')
        fields['motor_type'] = ttk.Combobox(dialog, width=18, values=[
            'DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V',
            'DM6006', 'DM8006', 'DM8009', 'DM10010', 'DM10010L',
            'DMG6220', 'DMH3510', 'DMH6215'
        ])
        fields['motor_type'].set(motor_data.get('motor_type', 'DM4340'))
        fields['motor_type'].grid(row=2, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Kp:").grid(row=3, column=0, padx=10, pady=5, sticky='e')
        fields['kp'] = ttk.Entry(dialog, width=20)
        fields['kp'].insert(0, motor_data.get('kp', '1.0'))
        fields['kp'].grid(row=3, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Kd:").grid(row=4, column=0, padx=10, pady=5, sticky='e')
        fields['kd'] = ttk.Entry(dialog, width=20)
        fields['kd'].insert(0, motor_data.get('kd', '0.1'))
        fields['kd'].grid(row=4, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Speed:").grid(row=5, column=0, padx=10, pady=5, sticky='e')
        fields['speed'] = ttk.Entry(dialog, width=20)
        fields['speed'].insert(0, motor_data.get('speed', '1.0'))
        fields['speed'].grid(row=5, column=1, padx=10, pady=5)

        ttk.Label(dialog, text="Torque Threshold:").grid(row=6, column=0, padx=10, pady=5, sticky='e')
        fields['torque_threshold'] = ttk.Entry(dialog, width=20)
        fields['torque_threshold'].insert(0, motor_data.get('torque_threshold', '1.0'))
        fields['torque_threshold'].grid(row=6, column=1, padx=10, pady=5)

        def save_motor():
            # Update motor in config
            self.motor_config['motors'][motor_idx] = {
                'motor_type': fields['motor_type'].get(),
                'slave_id': fields['slave_id'].get(),
                'master_id': fields['master_id'].get(),
                'kp': fields['kp'].get(),
                'kd': fields['kd'].get(),
                'speed': fields['speed'].get(),
                'torque_threshold': fields['torque_threshold'].get()
            }
            self.save_motor_config()
            self.refresh_motor_list()
            self.log(f"Updated motor {fields['slave_id'].get()}")
            dialog.destroy()

        button_frame = ttk.Frame(dialog)
        button_frame.grid(row=7, column=0, columnspan=2, pady=20)

        ttk.Button(button_frame, text="Save", command=save_motor).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(side=tk.LEFT, padx=5)

    def remove_motor(self):
        """Remove selected motor"""
        selection = self.motor_tree.selection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a motor to remove")
            return

        if not messagebox.askyesno("Confirm", "Are you sure you want to remove this motor?"):
            return

        # Get selected motor data
        item = selection[0]
        values = self.motor_tree.item(item)['values']
        slave_id = values[0]

        # Remove from config
        self.motor_config['motors'] = [m for m in self.motor_config['motors']
                                        if m['slave_id'] != slave_id]
        self.save_motor_config()
        self.refresh_motor_list()
        self.log(f"Removed motor {slave_id}")

    def save_motor_config(self):
        """Save motor configuration to file"""
        try:
            with open(self.motor_config_file, 'w') as f:
                json.dump(self.motor_config, f, indent=2)
            self.log("Motor configuration saved")
        except Exception as e:
            self.log(f"Error saving motor config: {e}")
            messagebox.showerror("Error", f"Failed to save motor configuration: {e}")

    def quick_connect(self):
        """Quick connect to motors using default/saved port"""
        self.notebook.select(0)  # Switch to control tab
        self.connect_motors()

    # Motor Control Methods
    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect_motors(self):
        """Connect to motors via serial port"""
        if not HAS_DM_CAN:
            messagebox.showerror("Error", "DM_CAN module not available")
            return

        try:
            port = self.port_var.get()
            baudrate = int(self.baudrate_var.get())

            self.serial_port = serial.Serial(port, baudrate, timeout=0.5)
            self.motor_control = MotorControl(self.serial_port)

            # Add all configured motors
            for motor_data in self.motor_config['motors']:
                motor_type_str = motor_data.get('motor_type', 'DM4310')
                slave_id_str = motor_data.get('slave_id', '0x01')
                master_id_str = motor_data.get('master_id', '0x11')

                # Convert hex strings to int
                slave_id = int(slave_id_str, 16) if slave_id_str.startswith('0x') else int(slave_id_str)
                master_id = int(master_id_str, 16) if master_id_str.startswith('0x') else int(master_id_str)

                # Get motor type enum
                motor_type = self._get_motor_type_enum(motor_type_str)

                # Create motor (correct order: MotorType, SlaveID, MasterID)
                motor = Motor(motor_type, slave_id, master_id)
                self.motors[slave_id_str] = motor
                self.motor_control.addMotor(motor)

                # Initialize motor
                if self.motor_control.switchControlMode(motor, Control_Type.POS_VEL):
                    self.log(f"Motor {slave_id_str} switched to POS_VEL mode")

                self.motor_control.set_zero_position(motor)
                self.motor_control.save_motor_param(motor)
                self.motor_enabled[slave_id_str] = False
                self.last_positions[slave_id_str] = 0.0

            self.is_connected = True
            self.conn_status_label.config(text="Status: Connected", foreground="green")
            self.toolbar_status.config(text="● Connected", foreground="green")
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)

            self.log(f"Connected to {port}: {len(self.motors)} motors configured")
            self.populate_motor_controls()

        except Exception as e:
            self.log(f"Connection error: {e}")
            messagebox.showerror("Connection Error", str(e))

    def disconnect_motors(self):
        """Disconnect from motors"""
        try:
            # Disable all motors first
            for motor_id, motor in self.motors.items():
                try:
                    self.motor_control.disable(motor)
                    self.log(f"Disabled motor {motor_id}")
                except Exception as e:
                    self.log(f"Error disabling motor {motor_id}: {e}")

            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            self.motors = {}
            self.motor_control = None
            self.serial_port = None
            self.is_connected = False
            self.motor_enabled = {}
            self.last_positions = {}

            self.conn_status_label.config(text="Status: Not Connected", foreground="red")
            self.toolbar_status.config(text="● Not Connected", foreground="red")
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)

            self.log("Disconnected from serial port")
            self.clear_motor_controls()

        except Exception as e:
            self.log(f"Disconnect error: {e}")
            messagebox.showerror("Disconnect Error", str(e))

    def populate_motor_controls(self):
        """Populate individual motor control checkboxes"""
        self.clear_motor_controls()

        if not self.motors:
            return

        # Create header
        ttk.Label(self.motor_controls_frame, text="Motor ID",
                 font=('Arial', 9, 'bold')).grid(row=0, column=0, padx=10, pady=5)
        ttk.Label(self.motor_controls_frame, text="Type",
                 font=('Arial', 9, 'bold')).grid(row=0, column=1, padx=10, pady=5)
        ttk.Label(self.motor_controls_frame, text="Function",
                 font=('Arial', 9, 'bold')).grid(row=0, column=2, padx=10, pady=5)
        ttk.Label(self.motor_controls_frame, text="Enabled",
                 font=('Arial', 9, 'bold')).grid(row=0, column=3, padx=10, pady=5)

        # Create row for each motor
        row = 1
        self.motor_check_vars = {}

        # Determine function for each motor
        motor_functions = {}
        for joint_idx, motor_id in self.config.get('motorMapping', {}).items():
            motor_functions[motor_id] = f"Joint {int(joint_idx) + 1}"
        gripper_id = self.config.get('gripperMotorId', '')
        if gripper_id:
            motor_functions[gripper_id] = "Gripper"

        for motor_id in sorted(self.motors.keys()):
            motor_data = next((m for m in self.motor_config['motors']
                              if m['slave_id'] == motor_id), None)
            motor_type = motor_data.get('motor_type', 'Unknown') if motor_data else 'Unknown'
            function = motor_functions.get(motor_id, "Unassigned")

            ttk.Label(self.motor_controls_frame, text=motor_id).grid(row=row, column=0, padx=10, pady=2)
            ttk.Label(self.motor_controls_frame, text=motor_type).grid(row=row, column=1, padx=10, pady=2)

            # Highlight gripper in blue
            func_label = ttk.Label(self.motor_controls_frame, text=function)
            if function == "Gripper":
                func_label.config(foreground='blue', font=('Arial', 9, 'bold'))
            func_label.grid(row=row, column=2, padx=10, pady=2)

            enabled_var = tk.BooleanVar(value=False)
            self.motor_check_vars[motor_id] = enabled_var

            check = ttk.Checkbutton(self.motor_controls_frame, variable=enabled_var,
                                   command=lambda mid=motor_id, var=enabled_var: self.toggle_motor(mid, var))
            check.grid(row=row, column=3, padx=10, pady=2)

            row += 1

    def clear_motor_controls(self):
        """Clear motor control widgets"""
        for widget in self.motor_controls_frame.winfo_children():
            widget.destroy()

    def toggle_motor(self, motor_id, enabled_var):
        """Toggle individual motor enable/disable"""
        try:
            motor = self.motors.get(motor_id)
            if not motor:
                return

            if enabled_var.get():
                self.motor_control.enable(motor)
                self.motor_enabled[motor_id] = True
                self.log(f"Enabled motor {motor_id}")
            else:
                self.motor_control.disable(motor)
                self.motor_enabled[motor_id] = False
                self.log(f"Disabled motor {motor_id}")

        except Exception as e:
            self.log(f"Error toggling motor {motor_id}: {e}")
            messagebox.showerror("Motor Error", str(e))

    def enable_all_motors(self):
        """Enable all motors"""
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to motors first")
            return

        try:
            for motor_id, motor in self.motors.items():
                self.motor_control.enable(motor)
                self.motor_enabled[motor_id] = True
                if motor_id in self.motor_check_vars:
                    self.motor_check_vars[motor_id].set(True)

            self.log(f"Enabled all {len(self.motors)} motors")

        except Exception as e:
            self.log(f"Error enabling motors: {e}")
            messagebox.showerror("Motor Error", str(e))

    def disable_all_motors(self):
        """Disable all motors (GUI button handler)"""
        if not self.is_connected:
            return

        try:
            for motor_id, motor in self.motors.items():
                self.motor_control.disable(motor)
                self.motor_enabled[motor_id] = False
                if motor_id in self.motor_check_vars:
                    self.motor_check_vars[motor_id].set(False)

            self.log(f"🛑 EMERGENCY: Disabled all {len(self.motors)} motors")

        except Exception as e:
            self.log(f"Error disabling motors: {e}")
            messagebox.showerror("Motor Error", str(e))

    async def disable_all_motors_async(self):
        """Disable all motors (async version for WebSocket)"""
        if not self.is_connected:
            return {"status": "error", "message": "Not connected to serial port"}

        if not self.motors:
            return {"status": "error", "message": "No motors configured"}

        try:
            disabled_count = 0
            for motor_id, motor in self.motors.items():
                self.motor_control.disable(motor)
                self.motor_enabled[motor_id] = False
                if motor_id in self.motor_check_vars:
                    self.motor_check_vars[motor_id].set(False)
                disabled_count += 1

            self.log(f"🛑 EMERGENCY: Disabled all {disabled_count} motors via WebSocket")

            return {
                "status": "success",
                "message": f"Disabled {disabled_count} motors",
                "disabled_count": disabled_count
            }

        except Exception as e:
            self.log(f"Error disabling motors: {e}")
            return {"status": "error", "message": str(e)}

    # WebSocket Server Methods
    async def handle_websocket_message(self, message):
        """Handle WebSocket message (same logic as old server)"""
        try:
            data = json.loads(message)
            command = data.get('command')
            params = data.get('params', {})

            if command == 'set_config':
                return {"status": "success", "config": self.config}
            elif command == 'on_joint_update':
                joint_idx = params.get('jointIndex')
                angle_rad = params.get('angleRad')
                angle_deg = angle_rad * 180 / 3.14159
                self.log(f"Joint {joint_idx + 1} updated to {angle_deg:.2f}°")
                return {"status": "success", "joint": joint_idx, "angle": angle_rad}
            elif command == 'on_gripper_update':
                motor_id = params.get('motorId')
                angle_rad = params.get('angleRad')
                angle_deg = angle_rad * 180 / 3.14159
                self.log(f"Gripper (Motor {motor_id}) updated to {angle_deg:.2f}°")
                return {"status": "success", "motorId": motor_id, "angle": angle_rad}
            elif command == 'send_to_motors':
                return await self.send_to_motors_async(params.get('mappedAngles', {}))
            elif command == 'disable_all_motors':
                return await self.disable_all_motors_async()
            elif command == 'get_status':
                return {
                    "status": "success",
                    "is_connected": self.is_connected,
                    "has_dm_can": HAS_DM_CAN,
                    "num_motors": len(self.motors),
                    "motor_ids": list(self.motors.keys()),
                    "config": self.config
                }
            else:
                return {"status": "error", "message": f"Unknown command: {command}"}

        except Exception as e:
            return {"status": "error", "message": str(e)}

    async def send_to_motors_async(self, mapped_angles):
        """Send joint angles to motors (async version)"""
        if not self.is_connected:
            return {"status": "error", "message": "Not connected to serial port"}

        if not self.motors:
            return {"status": "error", "message": "No motors configured"}

        # Build reverse lookup: motor_id -> should_reverse
        motor_reverse_lookup = {}

        # Check joint motors
        for joint_idx_str, motor_id in self.config.get('motorMapping', {}).items():
            is_reversed = self.config.get('motorReverse', {}).get(joint_idx_str, False)
            motor_reverse_lookup[motor_id] = is_reversed

        # Check gripper motor
        gripper_id = self.config.get('gripperMotorId', '')
        if gripper_id:
            motor_reverse_lookup[gripper_id] = self.config.get('gripperReverse', False)

        results = {}
        for motor_id_str, angle_rad in mapped_angles.items():
            if motor_id_str not in self.motors:
                results[motor_id_str] = {"status": "error", "message": "Motor not configured"}
                continue

            # Apply motor reversal if configured
            actual_angle = angle_rad
            if motor_reverse_lookup.get(motor_id_str, False):
                actual_angle = -angle_rad
                self.log(f"Motor {motor_id_str} reversed: {angle_rad:.3f} → {actual_angle:.3f} rad")

            if not self._is_safe_position(motor_id_str, actual_angle):
                results[motor_id_str] = {"status": "error", "message": "Unsafe position"}
                self.log(f"Blocked unsafe position {actual_angle:.3f} for motor {motor_id_str}")
                continue

            motor = self.motors[motor_id_str]

            try:
                self.log(f"[TRACE] Motor {motor_id_str}: Received angle_rad={angle_rad:.3f}, actual_angle={actual_angle:.3f}")

                self.motor_control.enable(motor)
                self.motor_control.control_Pos_Vel(motor, actual_angle, 10.0)
                self.last_positions[motor_id_str] = actual_angle

                self.motor_control.refresh_motor_status(motor)
                current_pos = motor.getPosition()

                results[motor_id_str] = {
                    "status": "success",
                    "target": float(actual_angle),
                    "current": float(current_pos)
                }

                self.log(f"Motor {motor_id_str}: commanded {actual_angle:.3f} rad, current {current_pos:.3f} rad")

            except Exception as e:
                results[motor_id_str] = {"status": "error", "message": str(e)}
                self.log(f"Error sending to motor {motor_id_str}: {e}")

        return {"status": "success", "results": results}

    def _is_safe_position(self, motor_id_str, new_position):
        """Check if position is safe"""
        # Skip safety checks for gripper/end effector - it can move through large angles
        gripper_motor_id = self.config.get('gripperMotorId', '')
        if motor_id_str == gripper_motor_id:
            return True

        if abs(new_position) > self.position_limit:
            self.log(f"WARNING: Position {new_position:.3f} exceeds limit ±{self.position_limit}")
            return False

        if motor_id_str in self.last_positions:
            last_pos = self.last_positions[motor_id_str]
            position_jump = abs(new_position - last_pos)

            if position_jump > self.max_position_jump:
                self.log(f"DANGER: Large position jump {position_jump:.3f} rad (max: {self.max_position_jump})")
                return False

        return True

    def _get_motor_type_enum(self, type_string):
        """Convert motor type string to enum"""
        motor_type_map = {
            "DM4310": DM_Motor_Type.DM4310,
            "DM4310_48V": DM_Motor_Type.DM4310_48V,
            "DM4340": DM_Motor_Type.DM4340,
            "DM4340_48V": DM_Motor_Type.DM4340_48V,
            "DM6006": DM_Motor_Type.DM6006,
            "DM8006": DM_Motor_Type.DM8006,
            "DM8009": DM_Motor_Type.DM8009,
            "DM10010": DM_Motor_Type.DM10010,
            "DM10010L": DM_Motor_Type.DM10010L,
            "DMG6220": DM_Motor_Type.DMG6220,
            "DMH3510": DM_Motor_Type.DMH3510,
            "DMH6215": DM_Motor_Type.DMH6215
        }
        return motor_type_map.get(type_string, DM_Motor_Type.DM4310)

    async def handle_client(self, websocket, path):
        """Handle WebSocket client connections"""
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.connected_clients.add(client_id)
        self.log(f"Client connected: {client_id}")
        self.update_client_list()

        # Send initial config
        await websocket.send(json.dumps({
            "command": "config_update",
            "config": self.config
        }))

        try:
            async for message in websocket:
                response = await self.handle_websocket_message(message)
                await websocket.send(json.dumps(response))

        except websockets.exceptions.ConnectionClosed:
            self.log(f"Client disconnected: {client_id}")
        except Exception as e:
            self.log(f"Error handling client {client_id}: {e}")
        finally:
            self.connected_clients.discard(client_id)
            self.update_client_list()

    async def broadcast_config(self):
        """Broadcast configuration to all connected clients"""
        if self.connected_clients:
            message = json.dumps({
                "command": "config_update",
                "config": self.config
            })
            await asyncio.gather(
                *[client.send(message) for client in self.connected_clients],
                return_exceptions=True
            )

    async def start_websocket_server(self):
        """Start WebSocket server"""
        try:
            self.log(f"Starting WebSocket server on port {self.websocket_port}...")
            async with websockets.serve(self.handle_client, "localhost", self.websocket_port):
                self.log(f"WebSocket server running on ws://localhost:{self.websocket_port}")
                self.root.after(0, lambda: self.server_status_label.config(
                    text=f"WebSocket Server: Running on port {self.websocket_port}",
                    foreground="green"
                ))
                await asyncio.Future()  # Run forever
        except Exception as e:
            self.log(f"WebSocket server error: {e}")
            self.root.after(0, lambda: self.server_status_label.config(
                text=f"WebSocket Server: Error - {e}",
                foreground="red"
            ))

    def update_client_list(self):
        """Update the connected clients display"""
        self.clients_text.config(state=tk.NORMAL)
        self.clients_text.delete(1.0, tk.END)
        if self.connected_clients:
            for client in self.connected_clients:
                self.clients_text.insert(tk.END, f"{client}\n")
        else:
            self.clients_text.insert(tk.END, "No clients connected\n")
        self.clients_text.config(state=tk.DISABLED)

    # Logging
    def log(self, message):
        """Log message to server log"""
        timestamp = __import__('datetime').datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        print(log_message)

        # Update GUI log
        if hasattr(self, 'log_text'):
            self.root.after(0, lambda: self._append_log(log_message))

    def _append_log(self, message):
        """Append message to log text widget"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    # Main Run Methods
    def run_websocket_server_thread(self):
        """Run WebSocket server in asyncio event loop"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_websocket_server())

    def run(self):
        """Run the application"""
        # Start WebSocket server in separate thread
        self.loop = asyncio.new_event_loop()
        self.server_thread = threading.Thread(target=self.run_websocket_server_thread, daemon=True)
        self.server_thread.start()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start Tkinter main loop
        self.log("Application started")
        self.root.mainloop()

    def on_closing(self):
        """Handle application closing"""
        if self.is_connected:
            self.disconnect_motors()

        self.log("Application closing...")
        self.root.destroy()


if __name__ == "__main__":
    app = RobotArmControlApp()
    app.run()
