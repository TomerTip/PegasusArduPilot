import threading
import time
import carb
import omni
from omni.isaac.sensor import _sensor

event_count = 0
start_time = time.time()
reading_buffer = []
physics_step_event = threading.Event()
stop_event = threading.Event()

imu = _sensor.acquire_imu_sensor_interface()

PHYSICS_STEP_FREQUENCY = 1/60
ARDUPILOT_FREQUENCY = 1/800

def get_imu_reading(reading):
        data = {}
        data["lin_acc_x"] = float(reading.lin_acc_x) 
        data["lin_acc_y"] = float(reading.lin_acc_y) 
        data["lin_acc_z"] = float(reading.lin_acc_z)
        data["ang_vel_x"] = float(reading.ang_vel_x) 
        data["ang_vel_y"] = float(reading.ang_vel_y) 
        data["ang_vel_z"] = float(reading.ang_vel_z) 
        data["orientation_x"] = float(reading.orientation[0]) 
        data["orientation_y"] = float(reading.orientation[1]) 
        data["orientation_z"] = float(reading.orientation[2]) 
        data["orientation_w"] = float(reading.orientation[3]) 
        return data

def step(dt: float , *args):
    imu_prim = "/Nova_Carter/chassis_link/chassis_imu/Imu_Sensor"
    global start_time
    global event_count
    global reading_buffer

    if physics_step_event.is_set():
        sensor_data = imu.get_sensor_reading(imu_prim, use_latest_data = True)
        reading_buffer.append(sensor_data)
        physics_step_event.clear()

    #every second, carb.log_warn the elapsed time and the event count
    if dt > PHYSICS_STEP_FREQUENCY:
        carb.log_warn(f"Elapsed time: {dt:.2f}s, Event count: {event_count}, Frequency: {event_count / dt:.2f} Hz, dt: {dt:.2f}s, total readings: {len(reading_buffer)}")
        if len(reading_buffer) > 0:
            carb.log_warn(get_imu_reading(reading_buffer[-1]))
        
        start_time = time.time()
        event_count = 0
        reading_buffer = []

    event_count += 1

def loop():
    global start_time

    while event_count < 1000:
        dt = time.time() - start_time    
        step(dt)
        time.sleep(ARDUPILOT_FREQUENCY)

def physics_step(dt:float , *args):
    physics_step_event.set()


print("Starting thread")
worker = threading.Thread(target=loop)
worker.start()

# stop_event.set()

carb.log_warn("Done")

physx = omni.physx.get_physx_interface()
physx_sub = physx.subscribe_physics_step_events(physics_step)
# del physx_sub
# worker.join()

carb.log_info("Done")