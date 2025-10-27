# ros_interface.py
import csv
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray

class ROSInterface(Node):
    def __init__(self):
        super().__init__('vending_debug_ui_node')

        # ========== Core publishers (all topics) ==========
        self.agitator_pub = self.create_publisher(Int32, '/Agitator_control', 10)
        self.linear_pub = self.create_publisher(Float32MultiArray, '/target_xy_mm', 10)

        self.publisher_motor_command = self.create_publisher(Int32MultiArray, '/motor_command', 10)
        self.publisher_ac = self.create_publisher(Int32MultiArray, '/ac', 10)
        self.publisher_door = self.create_publisher(Int32MultiArray, '/target_pose_door', 10)
        self.publisher_ac_pump = self.create_publisher(Int32MultiArray, '/ac_pump', 10)
        self.dc_pump_pub = self.create_publisher(Int32MultiArray, '/dc_pump', 10)
        self.conveyor_pub = self.create_publisher(Int32, '/set_conveyor_distance', 10)
        self.target_position_pub = self.create_publisher(Int32MultiArray, '/target_position_array', 10)
        self.blender_pose_pub = self.create_publisher(Int32MultiArray, '/target_blender_pose', 10)
        self.play_command_pub = self.create_publisher(Int32, '/play_command', 10)

        # ========== Distance arrays + subscribers ==========
        # distance -> 12 values
        self.distance_data = [0] * 12
        # distance_solid -> 6 values
        self.distance_solid_data = [0] * 6

        self.distance_sub = self.create_subscription(
            Int32MultiArray,
            '/distance',
            self.distance_callback,
            10
        )
        self.distance_solid_sub = self.create_subscription(
            Int32MultiArray,
            '/distance_solid',
            self.distance_solid_callback,
            10
        )

        # ========== Recorder ==========
        self.recording = False
        self.recorded_actions = []
        #self.temp_recorded_actions = []
        self.temp_last_action = None
        self.get_logger().info('ROSInterface initialized with publishers and subscribers.')

    # ==========================
    # 🎥 RECORDING CONTROL
    # ==========================
    def start_recording(self):
        self.recording = True
        self.recorded_actions = []
        self.get_logger().info("Recording started...")

    # def save_last_action(self):
    #     """Save the most recent action from temp to finalized recorded actions"""
    #     if self.temp_recorded_actions:
    #         last = self.temp_recorded_actions[-1]
    #         self.recorded_actions.append(last)
    #         self.get_logger().info(f"Saved action to recording: {last}")
    #     else:
    #         self.get_logger().info("No action to save.")

    def save_last_action(self):
        """Save the most recent action from temp to finalized recorded actions"""
        if self.temp_last_action:
            topic, data = self.temp_last_action
            self.recorded_actions.append([time.time(), topic, repr(data)])
            self.temp_last_action = None
            self.get_logger().info(f"Saved action: {topic} -> {data}")
        else:
            self.get_logger().info("No action to save.")    

    def stop_recording(self):
        self.recording = False
        filename = f"ros_sequence_{int(time.time())}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "topic", "data"])
            writer.writerows(self.recorded_actions)
        self.get_logger().info(f"Recording saved to {filename}")

    def record_action(self, topic: str, data):
        if self.recording:
            # Save timestamp (epoch), topic string, and repr of data
            # Use repr so complex lists serialize well
            self.recorded_actions.append([time.time(), topic, repr(data)])

    # ==========================
    # ▶️ PLAYBACK FUNCTION
    # ==========================
    def playback_sequence(self, filename: str):
        """
        Replay a previously recorded CSV sequence preserving original timing.
        """
        self.get_logger().info(f"Replaying sequence from {filename}...")
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                previous_time = None
                for row in reader:
                    topic = row["topic"]
                    data = eval(row["data"])
                    timestamp = float(row["timestamp"])

                    # Wait for the same amount of time as when recorded
                    if previous_time is not None:
                        time.sleep(timestamp - previous_time)
                    previous_time = timestamp

                    # Publish to the correct topic
                    if topic == "/ac":
                        self.publish_ac_command(data, record=False)
                    elif topic == "/ac_pump":
                        self.publish_ac_pump_command(data, record=False)
                    elif topic == "/dc_pump":
                        self.publish_dc_pump(data, record=False)
                    elif topic == "/motor_command":
                        self.publish_motor_command(int(data[0]), int(data[1]), record=False)
                    elif topic == "/target_pose_door":
                        self.publish_door_command(data, record=False)
                    elif topic == "/set_conveyor_distance":
                        self.publish_conveyor_distance(int(data), record=False)
                    elif topic == "/target_position_array":
                        self.publish_target_position_array(data, record=False)
                    elif topic == "/Agitator_control":
                        self.publish_agitator_control(int(data), record=False)
                    elif topic == "/target_xy_mm":
                        self.publish_linear_position(float(data[0]), float(data[1]), record=False)
                    elif topic == "target_blender_pose":
                        self.publish_blender_pose(int(data[0]), int(data[1]), record=False)  
                    elif topic == "/play_command":
                        self.publish_play_command(int(data), record=False)    

                    else:
                        self.get_logger().warn(f"Unknown topic in sequence: {topic}")

        except FileNotFoundError:
            self.get_logger().error(f"Playback file not found: {filename}")
        except Exception as e:
            self.get_logger().error(f"Playback error: {e}")

        self.get_logger().info("Playback finished ✅")    

    # ==========================
    # Distance callbacks
    # ==========================
    def distance_callback(self, msg: Int32MultiArray):
        # store as list for UI consumption
        self.distance_data = list(msg.data)

    def distance_solid_callback(self, msg: Int32MultiArray):
        self.distance_solid_data = list(msg.data)

    # ==========================
    # 📨 PUBLISHERS (with optional recording)
    # All publish methods accept a `record` flag used internally by playback
    # ==========================
    def publish_agitator_control(self, data_value: int, record: bool = True):
        msg = Int32()
        msg.data = int(data_value)
        self.agitator_pub.publish(msg)
        if record:
            self.temp_last_action = ("/Agitator_control", msg.data)
        self.get_logger().info(f'Published /Agitator_control: {msg.data}')

    def publish_linear_position(self, x_val: float, y_val: float, record: bool = True):
        msg = Float32MultiArray()
        msg.data = [float(x_val), float(y_val)]
        self.linear_pub.publish(msg)
        if record:
            self.temp_last_action = ("/target_xy_mm", list(msg.data))
        self.get_logger().info(f'Published /target_xy_mm: [x={x_val}, y={y_val}]')

    def publish_motor_command(self, dispenser_index: int, speed: int, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(dispenser_index), int(speed)]
        self.publisher_motor_command.publish(msg)
        if record:
            self.temp_last_action = ("/motor_command", list(msg.data))
        # log as list for readable output
        self.get_logger().info(f"Published /motor_command: {list(msg.data)}")

    def publish_ac_command(self, data: list, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.publisher_ac.publish(msg)
        if record:
            self.temp_last_action = ("/ac", list(msg.data))
        self.get_logger().info(f"Published /ac: {list(msg.data)}")

    def publish_ac_pump_command(self, data: list, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.publisher_ac_pump.publish(msg)
        if record:
            self.temp_last_action = ("/ac_pump", list(msg.data))
        self.get_logger().info(f"Published /ac_pump: {list(msg.data)}")

    def publish_dc_pump(self, data: list, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.dc_pump_pub.publish(msg)
        if record:
            self.temp_last_action = ("/dc_pump", list(msg.data))
        self.get_logger().info(f"Published /dc_pump: {list(msg.data)}")

    def publish_door_command(self, data: list, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.publisher_door.publish(msg)
        if record:
            self.temp_last_action = ("/target_pose_door", list(msg.data))
        self.get_logger().info(f"Published /target_pose_door: {list(msg.data)}")

    def publish_conveyor_distance(self, value: int, record: bool = True):
        msg = Int32()
        msg.data = int(value)
        self.conveyor_pub.publish(msg)
        if record:
            self.temp_last_action = ("/set_conveyor_distance", msg.data)
        self.get_logger().info(f"Published /set_conveyor_distance: {msg.data}")

    def publish_target_position_array(self, data: list, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        self.target_position_pub.publish(msg)
        if record:
            self.temp_last_action = ("/target_position_array", list(msg.data))
        self.get_logger().info(f"Published /target_position_array: {list(msg.data)}")

    def publish_blender_pose(self, back_front: int, up_down: int, record: bool = True):
        msg = Int32MultiArray()
        msg.data = [int(back_front), int(up_down)]
        self.blender_pose_pub.publish(msg)
        if record:
            self.temp_last_action = ("/target_blender_pose", list(msg.data))
        self.get_logger().info(f"Published /target_blender_pose: {list(msg.data)}")

    def publish_play_command(self, value: int, record: bool = True):
        msg = Int32()
        msg.data = int(value)
        self.play_command_pub.publish(msg)
        if record:
            self.temp_last_action = ("/play_command", msg.data)
        self.get_logger().info(f"Published /play_command: {msg.data}")


        


        
