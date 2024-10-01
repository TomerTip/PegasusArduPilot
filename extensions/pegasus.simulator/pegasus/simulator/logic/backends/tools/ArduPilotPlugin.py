import socket
import struct
import threading
import time
import json
import numpy as np
from dataclasses import dataclass
from typing import List
from pprint import pprint

class ArduPilotPlugin:
    SERVO_PACKET_SIZE = 40
    SERVO_PACKET_MAGIC = 0x481A
    AP_COMM_FREQUENCY = 1/1000 # hz109
    
    def __init__(self, sensor_rate: float):

        # The address for the flight dynamics model (i.e. this plugin)
        self.fdm_address = '127.0.0.1'
        # The port for the flight dynamics model
        self.fdm_port_in = 9002

        # FCU address and port are auto detected from receving UDP packet from connected client
        # The address for the SITL flight controller
        self.fcu_address = None
        # The port for the SITL flight controller
        self.fcu_port_out = None

        # Last received frame rate from the ArduPilot controller
        self.fcu_frame_rate = 0
        # Last received frame count from the ArduPilot controller
        self.fcu_frame_count = -1

        # Set to false when Gazebo starts to prevent blocking, true when
        # the ArduPilot controller is detected and online, and false if the
        # connection to the ArduPilot controller times out.
        self.arduPilotOnline = False
        
        # Number of consecutive missed ArduPilot controller messages
        self.connectionTimeoutCount = 0
        #  Max number of consecutive missed ArduPilot controller messages before timeout
        self.connectionTimeoutMaxCount = 10
        
        # Set true to enforce lock-step simulation
        self.isLockStep = False
               
        # Keep track of controller update sim-time.
        self.last_controller_update_time = 0
        # Keep track of the time the last servo packet was received.
        self.last_servo_packet_recv_time = 0
        
        # Sockets
        self.motor_control_sock = None
        
        # PWM packet buffer
        self.pwms = [0] * 16
        # Sensor Data from simulator
        self.sensor_data = {
            "sim_position": [0, 0, 0],
            "sim_attitude": [1, 0, 0, 0],
            "sim_velocity_inertial": [0, 0, 0],
            "xgyro": 0,
            "ygyro": 0,
            "zgyro": 0,
            "xacc": 0,
            "yacc": 0,
            "zacc": 0
        }

        # Last sent JSON string, so we can resend if needed.
        self.json_str: str = ""

        # Simulation time
        self.sim_time_start = time.time()
        
        # Creates a new thread that runs the run_plugin method when started
        self.plugin_thread = threading.Thread(target=self.run_plugin, args=())

        self.ap_thread_running = False

        self.sensor_rate = sensor_rate
        self.comm_rate = self.AP_COMM_FREQUENCY
        self.steps = round(self.sensor_rate / self.comm_rate)
        self.step = 0

        # Placeholder for previous and current sensor data dictionary
        self.prev_sensor_data = None
        self.curr_sensor_data = None
        self.time_since_last_update = 0.0

    
    def init_sockets(self):
        self.motor_control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_control_sock.setblocking(True)
        self.motor_control_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.motor_control_sock.bind((self.fdm_address, self.fdm_port_in))
            print(f"Flight dynamics model @ {self.fdm_address}:{self.fdm_port_in}")

        except socket.error as e:
            print(f"Failed to bind with {self.fdm_address}:{self.fdm_port_in}. Aborting plugin.")

    def create_state_json(self, sensor_data, sim_time):
        state = {
            "timestamp": sim_time,
            "imu": {
                "gyro": [
                    sensor_data["xgyro"],
                    sensor_data["ygyro"],
                    sensor_data["zgyro"]
                ],
                "accel_body": [
                    sensor_data["xacc"],
                    sensor_data["yacc"],
                    sensor_data["zacc"],
                ]
            },  
            "position": [
               sensor_data["sim_position"][0], # X
               sensor_data["sim_position"][1], # Y
               sensor_data["sim_position"][2], # Z
            ],
            "quaternion": [
               sensor_data["sim_attitude"][0], # W
               sensor_data["sim_attitude"][1], # X
               sensor_data["sim_attitude"][2], # Y
               sensor_data["sim_attitude"][3], # Z
            ],
            "velocity": [
              sensor_data["sim_velocity_inertial"][0], # X
              sensor_data["sim_velocity_inertial"][1], # Y
              sensor_data["sim_velocity_inertial"][2]  # Z
            ]
        }

        
        # pprint(f"\n{state}\n")

        json_str = json.dumps(state, separators=(',', ':'))
        json_str = "\n" + json_str + "\n"
        json_str = json_str.encode('utf-8')

        self.json_str = json_str
        return self.json_str 
    
    def send_state(self):
        if self.motor_control_sock:
            bytes_sent = self.motor_control_sock.sendto(
                self.json_str,
                (self.fcu_address, self.fcu_port_out)
            )

    def unpack_servo_packet(self, data):
        # Ensure the data length is valid
        if len(data) != self.SERVO_PACKET_SIZE:
            raise ValueError(f"Data length must be {self.SERVO_PACKET_SIZE} bytes, got {len(data)} bytes")
        
        # Define the format string according to the structure for little-endian
        # '<HHI16H' specifies:
        #   - < (little-endian byte order)
        #   - HH (2 uint16_t fields)
        #   - I (1 uint32_t field)
        #   - 16H (16 uint16_t fields for pwm values)
        format_string = '<HHI16H'
        
        try:
            # Unpack the data
            unpacked_data = struct.unpack(format_string, data)
            pkt_magic = unpacked_data[0]
            pkt_frame_rate = unpacked_data[1]
            pkt_frame_count = unpacked_data[2]
            pkt_pwm = unpacked_data[3:]
            
            if pkt_magic != self.SERVO_PACKET_MAGIC:
                print(f"Incorrect protocol magic {pkt_magic}, should be {self.SERVO_PACKET_MAGIC}")
                return None, None, None, None
            
            return pkt_magic, pkt_frame_rate, pkt_frame_count, pkt_pwm
        
        except struct.error as e:
            print(f"Unpacking error: {e}")
            return None, None, None, None

    def drain_unread_packets(self):
        """
        Drains all unread packets from the UDP socket.
        """

        # Set socket to non-blocking mode
        self.motor_control_sock.setblocking(False)
        while True:
            try:
                # Attempt to receive data
                data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)
                if (self.fcu_address is None) or (self.fcu_port_out is None):
                    self.fcu_address = client_addr
                    self.fcu_port_out = client_out

            except BlockingIOError:
                # No more data to read, exit the loop
                break
            except Exception as e:
                # Handle unexpected exceptions
                print(f"An error occurred while draining packets: {e}")
                break
        
        # Reset the socket to blocking mode
        self.motor_control_sock.setblocking(True)
        print("Drained all packets.")

    def receive_servo_packet(self):
        # Determine wait time based on whether ArduPilot is online
        wait_ms = 10 if self.arduPilotOnline else 1
        wait_sec = wait_ms / 1000.0

        pkt_frame_rate = 0
        pkt_frame_count = 0

        try:
            # Set socket timeout based on wait_ms
            self.motor_control_sock.settimeout(wait_sec)

            # Receive the data and get the client address and port
            data, (client_addr, client_out) = self.motor_control_sock.recvfrom(self.SERVO_PACKET_SIZE)

            # Store the FCU (ArduPilot SITL) address and port if not already set
            if self.fcu_address is None or self.fcu_port_out is None:
                self.fcu_address = client_addr
                self.fcu_port_out = client_out

            # Unpack the received packet
            pkt_magic, pkt_frame_rate, pkt_frame_count, *pkt_pwm = self.unpack_servo_packet(data)
            pkt_pwm = pkt_pwm[0]

            if pkt_magic is None:
                return False, ()
            
        except socket.timeout:
            if self.arduPilotOnline:
                self.connectionTimeoutCount += 1
                if self.connectionTimeoutCount > self.connectionTimeoutMaxCount:
                    self.connectionTimeoutCount = 0
                    if self.isLockStep:
                        print("Send State from receive_servo_packet")
                        self.send_state()
                    else:
                        print("Socket timeout")
                        self.arduPilotOnline = False
                        print(f"Broken ArduPilot connection, resetting motor control.")
            return False, ()

        # Handle ArduPilot online status
        if not self.arduPilotOnline:
            print(f"Connected to ArduPilot controller @ {self.fcu_address}:{self.fcu_port_out}")
            self.arduPilotOnline = True

        # Update frame rate
        self.fcu_frame_rate = pkt_frame_rate

        # Check for controller reset, duplicate frames, or skipped frames
        if pkt_frame_count < self.fcu_frame_count:
            print("ArduPilot controller has reset")
        elif pkt_frame_count == self.fcu_frame_count:
            print("Duplicate input frame")
            if self.isLockStep:
                self.send_state()
            return False, []
        elif pkt_frame_count != self.fcu_frame_count + 1 and self.arduPilotOnline:
            print(f"Missed {pkt_frame_count - self.fcu_frame_count} input frames")

        # Update frame count
        self.fcu_frame_count = pkt_frame_count

        # Reset the connection timeout count
        self.connectionTimeoutCount = 0    

        return True, pkt_pwm
    
    @property
    def sim_time(self):
        return time.time() - self.sim_time_start

    def pre_update(self):
        # Update the control surfaces
        recieved, self.pwms = self.receive_servo_packet()
        if recieved:
            self.last_servo_packet_recv_time = self.sim_time

        return recieved, self.pwms

    
    def get_servos(self):
        return self.pwms

    def update_sensor_data(self, 
                        sim_position: List[float], 
                        sim_attitude: List[float], 
                        sim_velocity_inertial: List[float], 
                        xgyro: float,
                        ygyro: float, 
                        zgyro: float, 
                        xacc: float,
                        yacc: float, 
                        zacc: float):
        sensor_data = {
            "sim_position": sim_position,
            "sim_attitude": sim_attitude,
            "sim_velocity_inertial": sim_velocity_inertial,
            "xgyro": xgyro,
            "ygyro": ygyro,
            "zgyro": zgyro,
            "xacc": xacc,
            "yacc": yacc,
            "zacc": zacc
        }
        self.prev_sensor_data = self.curr_sensor_data
        self.curr_sensor_data = sensor_data
        self.last_controller_update_time = self.sim_time
        self.time_since_last_update = 0.0  # Reset time since last update when new data arrives

    def send_state_interpolated(self, sensor_rate: float = 1/60, comm_rate: float = 1/800):
        steps = round(sensor_rate / comm_rate)
        for step in range(steps):
            intepolated_sensor_data = self.interpolate_sensor_data(step)
            # timestamp_step = self.last_controller_update_time + (comm_rate * step)
            # self.create_state_json(sim_time=self.sim_time, sensor_data=intepolated_sensor_data)
            self.create_state_json(sim_time=self.sim_time, sensor_data=self.curr_sensor_data)
            self.send_state()


    def post_update(self):
        if self.sim_time > self.last_controller_update_time and self.arduPilotOnline:
            self.create_state_json(sim_time=self.last_controller_update_time, sensor_data=self.curr_sensor_data)
            self.send_state()


    def interpolate_sensor_data(self, step):
        """
        Interpolates the sensor data based on the step size, which is the ratio of sensor_rate to comm_rate.

        Args:
            step (float): The step size for interpolation, calculated as sensor_rate/comm_rate.

        Returns:
            dict: The interpolated sensor data.
        """
        # Calculate interpolation factor based on the step size
        steps = self.sensor_rate / self.comm_rate
        alpha = step / steps # normalize

        # Initialize dictionary for storing interpolated data
        interpolated_data = self.curr_sensor_data

        if bool(self.curr_sensor_data) and bool(self.prev_sensor_data):
            for key in self.curr_sensor_data.keys():
                # Get previous and current values for the key
                prev_value = self.prev_sensor_data[key]
                curr_value = self.curr_sensor_data[key]

                # Check if values are lists or numpy arrays and handle accordingly
                if isinstance(prev_value, (list, np.ndarray)) and isinstance(curr_value, (list, np.ndarray)):
                    # Perform linear interpolation for each element
                    interpolated_value = [(1 - alpha) * prev + alpha * curr for prev, curr in zip(prev_value, curr_value)]
                else:
                    # Perform linear interpolation for scalar values
                    interpolated_value = (1 - alpha) * prev_value + alpha * curr_value

                # Store the interpolated value
                interpolated_data[key] = interpolated_value

        return interpolated_data
    

    def run_plugin(self, interpolated=True):
        self.init_sockets()
        self.drain_unread_packets()

        self.ap_thread_running = True
        
        messages = 0
        start_dt = self.sim_time
        while self.ap_thread_running:
            self.pre_update()
            
            if interpolated:
                if self.step < self.steps:
                    self.step += 1
                else:
                    self.step = 0 
                
                print(f"self.step:{self.step}")
                self.curr_sensor_data = self.interpolate_sensor_data(self.step)
                pprint(f"{self.curr_sensor_data}")
            
            self.post_update()

            time.sleep(self.comm_rate)
            
            messages += 1
            dt = self.sim_time - start_dt
            # print(f"self.sim_time: {self.sim_time}, start_dt:{start_dt}, dt:{dt}")
            if dt >= 1.0:
                print(f"send {messages} in {dt} seconds")
                start_dt = self.sim_time
                messages = 0
            
    def start(self):
        print("Ardupilot Plugin Started")
        self.plugin_thread.start()

    def stop(self):
        print("Ardupilot Plugin Stopped")
        self.ap_thread_running = False
        self.plugin_thread.join()

    # Mock Sensor Data
    @dataclass
    class SensorData:
        sim_position = [
            -6.874587183616472e-12 + np.random.normal(0, 0.01),
            -1.6699334495870352e-12 + np.random.normal(0, 0.01),
            -0.1949994162458527 + np.random.normal(0, 0.01)
        ]
        sim_attitude = [
            1,
            -4.281847537232883e-12 + np.random.normal(0, 0.001),
            1.7627199589076353e-11 + np.random.normal(0, 0.001),
            -4.281847537232883e-12 + np.random.normal(0, 0.001)
        ]
        sim_velocity_inertial = [
            4.4028248386528135e-12 + np.random.normal(0, 0.01),
            5.46182226507895e-12 + np.random.normal(0, 0.01),
            5.454422342398644e-18 + np.random.normal(0, 0.01)
        ]
        
if __name__ == '__main__':
    ap = ArduPilotPlugin()
    ap.init_sockets()
    ap.drain_unread_packets()

    ap.ap_thread_running = True
    while ap.ap_thread_running:
        ap.pre_update()
        ap.post_update()
        time.sleep(ap.comm_rate)