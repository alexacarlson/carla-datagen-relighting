#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    #sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.path.append(glob.glob('/home/alexandracarlson/Desktop/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import pdb
import carla
from carla import ColorConverter as cc

import random

#try:
#    import pygame
#except ImportError:
#    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue
import imageio

class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def save_image(image, fn):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]  
    #pdb.set_trace()
    imageio.imwrite(fn, array)  

def imu_callback(imu, fn):
    #print("IMU measure:\n"+str(imu)+'\n')
    #pdb.set_trace()
    imu_dict={}
    imu_dict['frame'] = imu.frame
    imu_dict['timestamp'] = imu.timestamp
    imu_dict['transform'] = ','.join([str(j) for j in [imu.transform.location.x, imu.transform.location.y, imu.transform.location.z, imu.transform.rotation.pitch, imu.transform.rotation.yaw, imu.transform.rotation.roll] ])
    imu_dict['accelerometer'] = ','.join([str(j) for j in [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]])
    imu_dict['gyroscope']  = ','.join([str(j) for j in [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]])
    imu_dict['compass'] = imu.compass
    #csv_columns = ['frame','timestamp','transform', 'accelerometer', 'gyroscope', 'compass']
    #csv_file = os.path.join(output_dir, "IMU/%.6d.txt"%imu.frame)
    with open(fn, 'w') as ff:
        ff.write('\n'.join(["%s: %s"%(key_, imu_dict[key_]) for key_ in imu_dict]))

def save_lidar(lidar, fn):
    lidar.save_to_disk(fn)

#def draw_image(surface, image, blend=False):
#    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#    array = np.reshape(array, (image.height, image.width, 4))
#    array = array[:, :, :3]
#    array = array[:, :, ::-1]
#    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
#    if blend:
#        image_surface.set_alpha(100)
#    surface.blit(image_surface, (0, 0))#
#
import time

def main():
    nframes = 120
    #town_id = 'Town01_Opt'
    #TOWNIDS=['Town06_Opt','Town01_Opt','Town02_Opt','Town03_Opt','Town04_Opt','Town05_Opt','Town07_Opt','Town08_Opt','Town09_Opt'] 
    TOWNIDS=['Town05_Opt','Town06_Opt','Town07_Opt','Town08_Opt','Town09_Opt'] 
    #TOWNIDS=['Town02_Opt','Town03_Opt','Town04_Opt','Town05_Opt','Town06_Opt','Town07_Opt'] 
    #TOWNIDS=['Town06_Opt','Town07_Opt'] 
    #TOWNIDS=[ 'Town09_Opt'] 
    episode=0
    actor_list = []
    #pygame.init()

    dataloc = '/mnt/ExtraSpace/CARLA-SUN-SYNC-6'
    #display = pygame.display.set_mode((800, 600),pygame.HWSURFACE | pygame.DOUBLEBUF)
    #font = get_font()
    #clock = pygame.time.Clock()

    #framecount=0
    client = carla.Client('localhost', 2000)
    client.set_timeout(1000.0)

    #sun_locs = [[95.0,10.0],
    #            [80.0,30.0],
    #            [60.0,60.0],
    #            [15.0,80.0],
    #            [0.0,75.0],
    #            [-15.0,80.0],
    #            [-60.0,60.0],
    #            [-80.0,30.0],
    #            [-95.0,10.0]]
    #sun_locs1 = np.int(np.dstack(np.meshgrid(np.linspace(40.,360.,3), np.linspace(20.,90.,3) )).reshape(-1,2))
    sun_locs1 = np.array([[40,20],
                        [80,40],
                        [120,80],
                        [160,60],
                        [0, 90],
                        [200,30],
                        [240,60],
                        [280,70],
                        [320,50],
                        [360,40]])
    sun_locs2 = np.array([
                        [320,50],
                        [360,40]])
    #pdb.set_trace()
    #sun_locs2 = np.array([[253.33333333,  90.        ],[360.        ,  90.        ]])
    #sun_locs2 = np.array([[253.33333333,90.], [360.,90.]])
    ## run this episode/trajectory for each sun location
    #
    for town_id in TOWNIDS:
        print(town_id)
        if town_id == 'Town05_Opt':
            sun_locs = sun_locs2
        else:
            sun_locs = sun_locs1

        for ii,eps_sunloc in enumerate(sun_locs):
            print('sunloc '+str(eps_sunloc))
            framecount=0
            world = client.load_world(town_id,reset_settings=True) 
            eps_az = float(eps_sunloc[0])
            eps_alt = float(eps_sunloc[1]) 
            #pdb.set_trace()
            ## get weather
            weather = carla.WeatherParameters(sun_azimuth_angle=eps_az, sun_altitude_angle=eps_alt)
            ## set the weather for the world
            world.set_weather(weather)      

            m = world.get_map()
            #start_pose = random.choice(m.get_spawn_points())
            start_pose = m.get_spawn_points()[0]
            waypoint = m.get_waypoint(start_pose.location)      

            blueprint_library = world.get_blueprint_library()       

            #ego_vehicle = world.spawn_actor(random.choice(blueprint_library.filter('vehicle.*')),start_pose)
            ego_vehicle = world.spawn_actor(blueprint_library.filter('vehicle.*')[0],start_pose)
            actor_list.append(ego_vehicle)
            ego_vehicle.set_simulate_physics(False)     

            output_dir=os.path.join(dataloc, town_id, 'episode'+str(episode), 'sunloc_'+','.join([str(sl) for sl in eps_sunloc]))
            os.makedirs(output_dir, exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Lidar'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Pose'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'IMU'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'RGB_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'RGB_R'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'semantic_segmentation_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Depth_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Depth_R'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Normals_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Normals_R'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'AmbientOcclusion'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Basecolor_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Specular_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Roughness_L'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Metallic_L'), exist_ok=True) 
            ################################################
            ## ADD CAMERAS/DATA COLLECTION TO EGO VEHICLE ##
            ################################################
            #
            ### imu ##
            imu_bp = world.get_blueprint_library().find('sensor.other.imu')
            imu_location = carla.Location(x=1.5, z=2.4)
            imu_rotation = carla.Rotation(0,0,0)
            imu_transform = carla.Transform(imu_location,imu_rotation)
            #imu_bp.set_attribute("sensor_tick",str(3.0))
            ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            #def imu_callback(imu):
            #    #print("IMU measure:\n"+str(imu)+'\n')
            #    imu_dict={}
            #    imu_dict['frame'] = imu.frame
            #    imu_dict['timestamp'] = imu.timestamp
            #    imu_dict['transform'] = ','.join([str(j) for j in [imu.transform.location.x, imu.transform.location.y, imu.transform.location.z, imu.transform.rotation.pitch, imu.transform.rotation.yaw, imu.transform.rotation.roll] ])
            #    imu_dict['accelerometer'] = ','.join([str(j) for j in [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]])
            #    imu_dict['gyroscope']  = ','.join([str(j) for j in [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]])
            #    imu_dict['compass'] = imu.compass
            #    #csv_columns = ['frame','timestamp','transform', 'accelerometer', 'gyroscope', 'compass']
            #    csv_file = os.path.join(output_dir, "IMU/%.6d.txt"%imu.frame)
            #    with open(csv_file, 'w') as ff:
            #        #f = csv.writer(ff)
            #        #for key_ in imu_dict.keys():
            #        #    f.writerow("%s,%s\n"%(key_,imu_dict[key_]))
            #        ff.write('\n'.join(["%s"%(imu_dict[key_]) for key_ in imu_dict]))
            #ego_imu.listen(lambda imu: imu_callback(imu))
            ##
            ### lidar ## 
            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels',str(64))
            lidar_bp.set_attribute('points_per_second',str(100000))
            lidar_bp.set_attribute('rotation_frequency',str(40))
            lidar_bp.set_attribute('range',str(100))
            lidar_location =carla.Location(x=1.5, z=2.4) # carla.Location(0,0,2.5)
            lidar_rotation = carla.Rotation(0,0,0)
            lidar_transform = carla.Transform(lidar_location,lidar_rotation)
            lidar_cam = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
            actor_list.append(lidar_cam)
            #lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk(output_dir+'/Lidar/%.6d.ply' % point_cloud.frame))
            #
            ## stereo left camera ##
            #camera_transform_L = carla.Transform(carla.Location(x=1.5, z=2.4))#
            camera_transform_L = carla.Transform(carla.Location(x=1.5, z=2.4, y = 0.15))#
            #camera_transform_L = carla.Transform(carla.Location(x=0., y=0., z=0.))#
            ## Let's add now a "rgb" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_rgb_L = blueprint_library.find('sensor.camera.rgb')
            camera_rgb_L = world.spawn_actor(camera_bp_rgb_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_rgb_L)
            #print('created %s' % camera_rgb_L.type_id) 
            ##camera_Lgb_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/RGB/%s_%06d_%s.png'%(str(episode), image.frame, str(eps_sunloc))))
            #camera_rgb_L.listen(lambda image: image.save_to_disk(output_dir+'/RGB/%06d_rgb.png'%image.frame))
            #
            ## Let's add now a "lidar reflectance" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_lidarrefl_L = blueprint_library.find('sensor.camera.lidar_reflectance')
            camera_lidarrefl_L = world.spawn_actor(camera_bp_lidarrefl_L , camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_lidarrefl_L)
            #print('created %s' % camera_semantic_segmentation_L.type_id) 
            #camera_semantic_segmentation_L.listen(lambda image: image.save_to_disk(output_dir+'/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
            #
            ## Let's add now a "semantic_segmentation" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_semantic_segmentation_L = blueprint_library.find('sensor.camera.semantic_segmentation')
            camera_semantic_segmentation_L = world.spawn_actor(camera_bp_semantic_segmentation_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_semantic_segmentation_L)
            #print('created %s' % camera_semantic_segmentation_L.type_id) 
            #camera_semantic_segmentation_L.listen(lambda image: image.save_to_disk(output_dir+'/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
            #
            ## Let's add now a "depth" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_depth_L = blueprint_library.find('sensor.camera.depth')
            camera_depth_L = world.spawn_actor(camera_bp_depth_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_depth_L)
            #print('created %s' % camera_depth_L.type_id)    
            ## Now we register the function that will be called each time the sensor receives an image. In this example we are saving the image to disk converting the pixels to gray-scale.
            ##ccdep = carla.ColorConverter.LogarithmicDepth
            ##camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Depth/%06d_depth.png'%image.frame, ccdep))   
            #camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/Depth/%06d_depth.png'%image.frame))   
            #
            ## add normal map camera
            normalcam_bp_L = blueprint_library.find('sensor.camera.normal_map')
            normalcam_L = world.spawn_actor(normalcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(normalcam_L)
            #print('created %s'%normalcam_L.type_id)
            #normalcam.listen(lambda image: image.save_to_disk('/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/carla_out/%06d_normal.png' % image.frame))  
            #normalcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Normals/%06d_normal.png' % image.frame))  #
            #
            ## add ambient occlusion map camera
            #aocam_bp_L = blueprint_library.find('sensor.camera.ambientocclusion_map')
            #aocam_L = world.spawn_actor(aocam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(aocam_L)
            #print('created %s'%bccam_L.type_id)
            #bccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #
            #
            ## add basecolor map camera
            bccam_bp_L = blueprint_library.find('sensor.camera.mat_map')
            bccam_L = world.spawn_actor(bccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(bccam_L)
            #print('created %s'%bccam_L.type_id)
            #bccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #
            #
            ## add specular map camera
            speccam_bp_L = blueprint_library.find('sensor.camera.specular')
            speccam_L = world.spawn_actor(speccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(speccam_L)
            #print('created %s'%speccam_L.type_id)
            #speccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Specular/%06d_specularfromlighting.png' % image.frame))  #
            #
            ## add roughness map camera
            roughcam_bp_L = blueprint_library.find('sensor.camera.roughness')
            roughcam_L = world.spawn_actor(roughcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(roughcam_L)
            #print('created %s'%roughcam_L.type_id)
            #roughcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Roughness/%06d_roughness.png' % image.frame))    #
            #
            ## add metallic map camera
            metcam_bp_L = blueprint_library.find('sensor.camera.metallic')
            metcam_L = world.spawn_actor(metcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(metcam_L)
            #print('created %s'%metcam_L.type_id)
            #metcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Metallic/%06d_metallic.png' % image.frame))#     

            ## stereo r\ight camera ##
            #camera_transform_R = carla.Transform(carla.Location(x=1.5, z=2.4))#
            camera_transform_R = carla.Transform(carla.Location(x=1.5, z=2.4, y = -0.15))#
            #camera_transform_L = carla.Transform(carla.Location(x=0., y=0., z=0.))#
            ## Let's add now a "rgb" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_rgb_R = blueprint_library.find('sensor.camera.rgb')
            camera_rgb_R = world.spawn_actor(camera_bp_rgb_R, camera_transform_R, attach_to=ego_vehicle)
            actor_list.append(camera_rgb_R)
            #print('created %s' % camera_rgb_L.type_id) 
            ##camera_Lgb_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/RGB/%s_%06d_%s.png'%(str(episode), image.frame, str(eps_sunloc))))
            #camera_rgb_L.listen(lambda image: image.save_to_disk(output_dir+'/RGB/%06d_rgb.png'%image.frame))
            #
            ## Let's add now a "lidar reflectance" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            #camera_bp_lidarrefl_R = blueprint_library.find('sensor.camera.lidar_reflectance')
            #camera_lidarrefl_L = world.spawn_actor(camera_bp_lidarrefl_L , camera_transform_L, attach_to=ego_vehicle)
            #actor_list.append(camera_lidarrefl_L)
            #print('created %s' % camera_semantic_segmentation_L.type_id) 
            #camera_semantic_segmentation_L.listen(lambda image: image.save_to_disk(output_dir+'/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
            #
            ## Let's add now a "semantic_segmentation" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            #camera_bp_semantic_segmentation_L = blueprint_library.find('sensor.camera.semantic_segmentation')
            #camera_semantic_segmentation_L = world.spawn_actor(camera_bp_semantic_segmentation_L, camera_transform_L, attach_to=ego_vehicle)
            #actor_list.append(camera_semantic_segmentation_L)
            #print('created %s' % camera_semantic_segmentation_L.type_id) 
            #camera_semantic_segmentation_L.listen(lambda image: image.save_to_disk(output_dir+'/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
            #
            ## Let's add now a "depth" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_depth_R = blueprint_library.find('sensor.camera.depth')
            camera_depth_R = world.spawn_actor(camera_bp_depth_R, camera_transform_R, attach_to=ego_vehicle)
            actor_list.append(camera_depth_R)
            #print('created %s' % camera_depth_L.type_id)    
            ## Now we register the function that will be called each time the sensor receives an image. In this example we are saving the image to disk converting the pixels to gray-scale.
            ##ccdep = carla.ColorConverter.LogarithmicDepth
            ##camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Depth/%06d_depth.png'%image.frame, ccdep))   
            #camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/Depth/%06d_depth.png'%image.frame))   
            #
            ## add normal map camera
            normalcam_bp_R = blueprint_library.find('sensor.camera.normal_map')
            normalcam_R = world.spawn_actor(normalcam_bp_R, camera_transform_R , attach_to=ego_vehicle)
            actor_list.append(normalcam_R)
            #print('created %s'%normalcam_L.type_id)
            #normalcam.listen(lambda image: image.save_to_disk('/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/carla_out/%06d_normal.png' % image.frame))  
            #normalcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Normals/%06d_normal.png' % image.frame))  #
            #
            ## add ambient occlusion map camera
            #aocam_bp_L = blueprint_library.find('sensor.camera.ambientocclusion_map')
            #aocam_L = world.spawn_actor(aocam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(aocam_L)
            #print('created %s'%bccam_L.type_id)
            #bccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #
            #
            ## add basecolor map camera
            #bccam_bp_L = blueprint_library.find('sensor.camera.mat_map')
            #bccam_L = world.spawn_actor(bccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(bccam_L)
            #print('created %s'%bccam_L.type_id)
            #bccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #
            #
            ## add specular map camera
            #speccam_bp_L = blueprint_library.find('sensor.camera.specular')
            #speccam_L = world.spawn_actor(speccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(speccam_L)
            #print('created %s'%speccam_L.type_id)
            #speccam_L.listen(lambda image: image.save_to_disk(output_dir+'/Specular/%06d_specularfromlighting.png' % image.frame))  #
            #
            ## add roughness map camera
            #roughcam_bp_L = blueprint_library.find('sensor.camera.roughness')
            #roughcam_L = world.spawn_actor(roughcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(roughcam_L)
            ##print('created %s'%roughcam_L.type_id)
            ##roughcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Roughness/%06d_roughness.png' % image.frame))    #
            #
            ## add metallic map camera
            #metcam_bp_L = blueprint_library.find('sensor.camera.metallic')
            #metcam_L = world.spawn_actor(metcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            #actor_list.append(metcam_L)
            ##print('created %s'%metcam_L.type_id)
            ##metcam_L.listen(lambda image: image.save_to_disk(output_dir+'/Metallic/%06d_metallic.png' % image.frame))#  

            fullptcld = []
            # Create a synchronous mode context.
            with CarlaSyncMode(world, ego_imu, lidar_cam, camera_rgb_L, camera_rgb_R, camera_semantic_segmentation_L, camera_depth_L, camera_depth_R, normalcam_L, normalcam_R, bccam_L, speccam_L, roughcam_L, metcam_L, camera_lidarrefl_L, fps=30) as sync_mode:
                #with CarlaSyncMode(world, camera_rgb_L, camera_semantic_segmentation_L, camera_depth_L, normalcam_L, bccam_L, speccam_L, roughcam_L, metcam_L, fps=30) as sync_mode:
                while framecount<nframes:
                    #if should_quit():
                    #    return
                    #clock.tick()       

                    # Advance the simulation and wait for the data.
                    #snapshot, image_rgb, image_semseg = sync_mode.tick(timeout=2.0)
                    data = sync_mode.tick(timeout=10000.0)
                    #print(data[0].frame)
                    curr_frame = data[0].frame
                    curr_IMU = data[1]
                    imu_callback(curr_IMU, output_dir+"/IMU/%.6d.txt"%curr_frame)
                    curr_Lidar = data[2]
                    save_lidar(curr_Lidar, output_dir+"/Lidar/%.6d.txt"%curr_frame)
                    #pdb.set_trace()
                    curr_RGB = data[3]
                    save_image(curr_RGB, output_dir+'/RGB_L/%06d.png'%curr_frame)
                    curr_RGBR = data[4]
                    save_image(curr_RGBR, output_dir+'/RGB_R/%06d.png'%curr_frame)
                    curr_Semseg = data[5]
                    save_image(curr_Semseg, output_dir+'/semantic_segmentation_L/%06d.png'%curr_frame)
                    curr_Depth = data[6]
                    save_image(curr_Depth, output_dir+'/Depth_L/%06d.png'%curr_frame)
                    curr_DepthR = data[7]
                    save_image(curr_DepthR, output_dir+'/Depth_R/%06d.png'%curr_frame)
                    curr_Normal = data[8]
                    save_image(curr_Normal, output_dir+'/Normals_L/%06d.png'%curr_frame)
                    curr_NormalR = data[9]
                    save_image(curr_NormalR, output_dir+'/Normals_R/%06d.png'%curr_frame)
                    #curr_Amboccl = data[7]
                    #save_image(curr_Amboccl, output_dir+'/AmbientOcclusion/%06d.png'%curr_frame)
                    curr_Basecol = data[10]
                    save_image(curr_Basecol, output_dir+'/Basecolor_L/%06d.png'%curr_frame)
                    curr_Specular = data[11]
                    save_image(curr_Specular, output_dir+'/Specular_L/%06d.png'%curr_frame)
                    curr_Rough = data[12]
                    save_image(curr_Rough, output_dir+'/Roughness_L/%06d.png'%curr_frame)
                    curr_Metallic = data[13]
                    save_image(curr_Metallic, output_dir+'/Metallic_L/%06d.png'%curr_frame)
                    curr_lidarrefl = data[14]
                    save_image(curr_lidarrefl, output_dir+'/Lidar/%06d_refl_L.png'%curr_frame)

                    np.save(output_dir+'/Pose/%06d_camera2world_L.npy'%curr_frame, camera_rgb_L.get_transform().get_matrix())
                    np.save(output_dir+'/Pose/%06d_world2camera_L.npy'%curr_frame, camera_rgb_L.get_transform().get_inverse_matrix())
                    np.save(output_dir+'/Pose/%06d_camera2world_R.npy'%curr_frame, camera_rgb_R.get_transform().get_matrix())
                    np.save(output_dir+'/Pose/%06d_world2camera_R.npy'%curr_frame, camera_rgb_R.get_transform().get_inverse_matrix())
                    np.save(output_dir+'/Pose/%06d_lidar2world.npy'%curr_frame, lidar_cam.get_transform().get_matrix())
                    np.save(output_dir+'/Pose/%06d_world2lidar.npy'%curr_frame, lidar_cam.get_transform().get_inverse_matrix())
                    lidarpts = np.copy(np.frombuffer(data[2].raw_data, dtype=np.dtype('f4')))
                    lidarpts = np.reshape(lidarpts, (len(data[2]), 4))
                    local_lidar_points = np.array(lidarpts[:, :3]).T
                    intensity = np.array(lidarpts[:, 3])
                    local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]
                    world_points = np.dot(lidar_cam.get_transform().get_matrix(), local_lidar_points)
                    world_points = np.hstack([world_points[:3,:].T,intensity[:,None]])
                    fullptcld.append(world_points)
                    #pdb.set_trace()
                    #   
                    #from matplotlib import cm
                    #from PIL import Image
                    #VIRIDIS = np.array(cm.get_cmap('viridis').colors)
                    #VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
                    #image_w = 800 #camera_rgb_L.get_attribute("image_size_x").as_int()
                    #image_h = 600 #camera_rgb_L.get_attribute("image_size_y").as_int()
                    #fov = 90 #camera_bp.get_attribute("fov").as_float()
                    #focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))
                    ## In this case Fx and Fy are the same since the pixel aspect
                    ## ratio is 1
                    #K = np.identity(3)
                    #K[0, 0] = K[1, 1] = focal
                    #K[0, 2] = image_w / 2.0
                    #K[1, 2] = image_h / 2.0
                    #image_data = data[3]
                    #im_array = np.copy(np.frombuffer(image_data.raw_data, dtype=np.dtype("uint8")))
                    #im_array = np.reshape(im_array, (image_data.height, image_data.width, 4))
                    #im_array = im_array[:, :, :3][:, :, ::-1]
                    #lidar_data = data[2]
                    #p_cloud_size = len(lidar_data)
                    #p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                    #p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))
                    ## Lidar intensity array of shape (p_cloud_size,) but, for now, let's
                    ## focus on the 3D points.
                    #intensity = np.array(p_cloud[:, 3])
                    ## Point cloud in lidar sensor space array of shape (3, p_cloud_size).
                    #local_lidar_points = np.array(p_cloud[:, :3]).T
                    ## Add an extra 1.0 at the end of each 3d point so it becomes of
                    ## shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
                    #local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]
                    ## This (4, 4) matrix transforms the points from lidar space to world space.
                    #lidar_2_world = lidar_cam.get_transform().get_matrix()
                    ## Transform the points from lidar space to world space.
                    #world_points = np.dot(lidar_2_world, local_lidar_points)
                    ## This (4, 4) matrix transforms the points from world to sensor coordinates.
                    #world_2_camera = np.array(camera_rgb_L.get_transform().get_inverse_matrix())
                    ## Transform the points from world space to camera space.
                    #sensor_points = np.dot(world_2_camera, world_points)
                    ## New we must change from UE4's coordinate system to an "standard"
                    ## camera coordinate system (the same used by OpenCV):
                    #point_in_camera_coords = np.array([
                    #    sensor_points[1],
                    #    sensor_points[2] * -1,
                    #    sensor_points[0]])
                    ## Finally we can use our K matrix to do the actual 3D -> 2D.
                    #points_2d = np.dot(K, point_in_camera_coords)
                    ## Remember to normalize the x, y values by the 3rd value.
                    #points_2d = np.array([
                    #    points_2d[0, :] / points_2d[2, :],
                    #    points_2d[1, :] / points_2d[2, :],
                    #    points_2d[2, :]])
                    ## At this point, points_2d[0, :] contains all the x and points_2d[1, :]
                    ## contains all the y values of our points. In order to properly
                    ## visualize everything on a screen, the points that are out of the screen
                    ## must be discarted, the same with points behind the camera projection plane.
                    #points_2d = points_2d.T
                    #intensity = intensity.T
                    #points_in_canvas_mask = \
                    #    (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
                    #    (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
                    #    (points_2d[:, 2] > 0.0)
                    #points_2d = points_2d[points_in_canvas_mask]
                    #intensity = intensity[points_in_canvas_mask]
                    ## Extract the screen coords (uv) as integers.
                    #u_coord = points_2d[:, 0].astype(np.int)
                    #v_coord = points_2d[:, 1].astype(np.int)
                    ## Since at the time of the creation of this script, the intensity function
                    ## is returning high values, these are adjusted to be nicely visualized.
                    #intensity = 4 * intensity - 3
                    #color_map = np.array([
                    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 0]) * 255.0,
                    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]) * 255.0,
                    #    np.interp(intensity, VID_RANGE, VIRIDIS[:, 2]) * 255.0]).astype(np.int).T
                    ## Draw the 2d points on the image as a single pixel using numpy.
                    #im_array2 = np.zeros_like(im_array)
                    #im_array2[v_coord, u_coord] = color_map
                    ## Save the image using Pillow module.
                    #image = Image.fromarray(np.hstack([im_array, im_array2]))
                    #image.save("%08d.png" % image_data.frame)

                    #pdb.set_trace()
                    # Choose the next waypoint and update the car location.
                    #waypoint = random.choice(waypoint.next(1.5))
                    waypoint = waypoint.next(1.5)[0]
                    ego_vehicle.set_transform(waypoint.transform)       

                    framecount+=1
                    #image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                    #fps = round(1.0 / snapshot.timestamp.delta_seconds)        

                    ## Draw the display.
                    #draw_image(display, image_rgb)
                    #draw_image(display, image_semseg, blend=True)
                    #display.blit(
                    #    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    #    (8, 10))
                    #display.blit(
                    #    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    #    (8, 28))
                    #pygame.display.flip()      
                ## save full point cloud at the end of the simulation
                np.save(output_dir+"/Lidar/fullptcld.npy", np.vstack(fullptcld))
            print('destroying actors.')
            #for actor in actor_list:
            #    actor.destroy()
                    ## destroy actors
            #pdb.set_trace()
            client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
            print('done.')
            #break
            time.sleep(0.5)
            #pygame.quit()
            #print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
