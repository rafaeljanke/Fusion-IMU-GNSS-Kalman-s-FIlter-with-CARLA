import carla
import random
import cv2
import numpy as np
import time
import csv
import pygame
from datetime import datetime


# Let's the define the headings of our csv file and save them.

headers = ['TimeStamp', 'Latitude', 'Longitude', 'Accelx', 'Accely', 'Accelz', 'Gyrox', 'Gyroy', 'Gyroz', 'GTx', 'GTy', 'GTz', 'GTPitch', 'GTRoll', 'GTYaw']

with open('data.csv','w') as file:
    writer = csv.writer(file)
    writer.writerow(headers)


clock = pygame.time.Clock()
client = carla.Client('localhost',2000)
world = client.load_world('Town06')
weather = carla.WeatherParameters(
    cloudiness=0.0,
    precipitation=0.0,
    sun_altitude_angle=10.0,
    sun_azimuth_angle = 70.0,
    precipitation_deposits = 0.0,
    wind_intensity = 0.0,
    fog_density = 0.0,
    wetness = 0.0,
)
world.set_weather(weather)

bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

vehicle_bp = bp_lib.find('vehicle.audi.etron')
ego_vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

spectator = world.get_spectator()
# transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),ego_vehicle.get_transform().rotation)
transform = carla.Transform(carla.Location(x=-124.322495, y=149.545090, z=104.983757), carla.Rotation(pitch=-37.868515, yaw=-155.810608, roll=0.000004))
#Location(x=-124.322495, y=149.545090, z=104.983757), Rotation(pitch=-37.868515, yaw=-155.810608, roll=0.000004)
spectator.set_transform(transform)

# for i in range(200):
#     vehicle_bp = random.choice(bp_lib.filter('vehicle'))
#     npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# for v in world.get_actors().filter('*vehicle*'):
#     v.set_autopilot(True)
ego_vehicle.set_autopilot(True)

# Add RGB camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(x =-0.1,z=1.7))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

# Storing image width and height values
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Add depth camera
depth_camera_bp = bp_lib.find('sensor.camera.depth')
depth_camera = world.spawn_actor(depth_camera_bp, camera_init_trans, attach_to=ego_vehicle)

# Add navigation sensor
gnss_bp = bp_lib.find('sensor.other.gnss')
gnss_sensor = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=ego_vehicle)

# Add IMU sensor
imu_bp = bp_lib.find('sensor.other.imu')
imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)

def rgb_callback(image, data_dict):
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) #Reshaping with alpha channel
    img[:,:,3] = 255 #Setting the alpha to 255
    data_dict['rgb_image'] = img

# Callback functions for the above sensors
def depth_callback(image, data_dict):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    data_dict['depth_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

def gnss_callback(data, data_dict):
    transform = ego_vehicle.get_transform()

    data_dict['gt'] = {
        'location': transform.location,
        'rotation': transform.rotation
    }

    #data_dict['gnss'] = [data.latitude, data.longitude]
    print(data)
    data_dict['gnss'] = {
        'timestamp': data.timestamp,
        'latitude': data.latitude,
        'longitude': data.longitude,
        'altitude': data.altitude,
        'frame': data.frame

    }

def imu_callback(data, data_dict):
    print(data)
    data_dict['imu'] = {
        'gyro': data.gyroscope,
        'accel': data.accelerometer,
        'compass': data.compass,
        'timestamp': data.timestamp,
        'frame': data.frame
    }

# Update the sensor_data dictionary to include the depth_image, gnss and imu keys and default values
initial_transform = carla.Transform(carla.Location(0., 0., 0.), carla.Rotation(0., 0., 0.))
sensor_data = {
    'rgb_image': np.zeros((image_h, image_w, 4)),
    'depth_image': np.zeros((image_h, image_w, 4)),
    'gt': {
        'location': initial_transform.location,
        'rotation': initial_transform.rotation
    },
    'gnss': {
        'timestamp': 0,
        'latitude': 0,
        'longitude': 0,
        'altitude': 0,
        'frame': 0
},
    'imu': {
        'gyro': carla.Vector3D(),
        'accel': carla.Vector3D(),
        'compass': 0
    }
}

# Listen to the sensor feed
#depth_camera.listen(lambda image: depth_callback(image, sensor_data))
gnss_sensor.listen(lambda event: gnss_callback(event, sensor_data))
imu_sensor.listen(lambda event: imu_callback(event, sensor_data))

# We'll add all the other sensors' data into this dictionary later.
# For now, we've added the camera feed
# sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4))}

camera.listen(lambda image: rgb_callback(image, sensor_data))

while True:
    spectator = world.get_spectator()
    print("spec:", spectator.get_transform())

    # Output camera display onto an OpenCV Window
    cv2.imshow("RGB_Image", sensor_data['rgb_image'])
    #cv2.imshow("Depth_Image", sensor_data['depth_image'])
    #print("gnss:", sensor_data['gnss'])
    #print("imu_gyro:", sensor_data['imu']['gyro'])
    #print("imu_accel:", sensor_data['imu']['accel'])
    #print("imu_compass:", sensor_data['imu']['compass'])
    #qprint("---")

    time_stamp = sensor_data['gnss']['timestamp']
    lat = sensor_data['gnss']['latitude']
    long = sensor_data['gnss']['longitude']

    #Before storing, we subtrract gravity
    accel = sensor_data['imu']['accel'] - carla.Vector3D(x=0,y=0,z=9.81)
    gyro = sensor_data['imu']['gyro']

    #print(f'AQUIIII {time_stamp}')
    gt_location = sensor_data['gt']['location']
    gt_rotation = sensor_data['gt']['rotation']
    all_sensor_data = [time_stamp,lat,long,accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z,
                       gt_location.x, gt_location.y, gt_location.z, gt_rotation.pitch,
                       gt_rotation.roll, gt_rotation.yaw]


    with open('data.csv', 'a') as file:
        writer = csv.writer(file)
        writer.writerow(all_sensor_data)

    clock.tick(60)


    if cv2.waitKey(1) == ord('q'):

        today = datetime.today()

        break