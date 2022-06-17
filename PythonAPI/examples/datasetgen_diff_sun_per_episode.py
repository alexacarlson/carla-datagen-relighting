#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import argparse
import pdb 
import numpy as np
import math

try:
    #sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
    sys.path.append(glob.glob('/home/alexandracarlson/Desktop/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

#try:
#    sys.path.append(glob.glob('PythonAPI')[0])
#except IndexError:
#    print("PythonAPI package wasn't found")#

#try:
#    sys.path.append(glob.glob('**/agents')[0])
#except IndexError:
#    print("PythonAPI/agents package wasn't found")#

#try:
#    sys.path.append(glob.glob('**/agents/navigation')[0])
#except IndexError:
#    print("PythonAPI/agents/navigation package wasn't found")#

#try:
#    sys.path.append(glob.glob('**/agents/tools')[0])
#except IndexError:
#    print("PythonAPI/agents/tools package wasn't found")#

#pdb.set_trace()

import carla
from carla import ColorConverter as cc

import random
import time
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
import csv
def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = 35.0 * (math.sin(self._t) + 1.0)

    def __str__(self):
        return 'Sun(%.2f, %.2f)' % (self.azimuth, self.altitude)


#class Storm(object):
#    def __init__(self, precipitation):
#        self._t = precipitation if precipitation > 0.0 else -50.0
#        self._increasing = True
#        self.clouds = 0.0
#        self.rain = 0.0
#        self.puddles = 0.0
#        self.wind = 0.0
#        #
#    def tick(self, delta_seconds):
#        delta = (1.3 if self._increasing else -1.3) * delta_seconds
#        self._t = clamp(delta + self._t, -250.0, 100.0)
#        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
#        self.rain = clamp(self._t, 0.0, 80.0)
#        delay = -10.0 if self._increasing else 90.0
#        self.puddles = clamp(self._t + delay, 0.0, 75.0)
#        self.wind = clamp(self._t - delay, 0.0, 80.0)
#        if self._t == -250.0:
#            self._increasing = True
#        if self._t == 100.0:
#            self._increasing = False
#            #
#    def __str__(self):
#        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        #self._storm = Storm(weather.precipitation)

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        #self._storm.tick(delta_seconds)
        #self.weather.cloudyness = self._storm.clouds
        #self.weather.precipitation = self._storm.rain
        #self.weather.precipitation_deposits = self._storm.puddles
        #self.weather.wind_intensity = self._storm.wind
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self):
        #return '%s %s' % (self._sun, self._storm)
        return '%s' % (self._sun)


def run_game(config, town_id):
    


    

    #speed_factor = 1.0#config.speed
    #update_freq = 0.1 / speed_factor
    #sun_locs = [[95.0,10.0],
    #            [80.0,30.0],
    #            [60.0,60.0],
    #            [15.0,80.0],
    #            [0.0,75.0],
    #            [-15.0,80.0],
    #            [-60.0,60.0],
    #            [-80.0,30.0],
    #            [-95.0,10.0]]
    sun_locs = np.dstack(np.meshgrid(np.linspace(40.,360.,4), np.linspace(10.,90.,4) )).reshape(-1,2)

    # First of all, we need to create the client that will send the requests
    # to the simulator. Here we'll assume the simulator is accepting
    # requests in the localhost at port 2000.
    client = carla.Client('localhost', 2000)
    client.set_timeout(1000.0)
    #pdb.set_trace()

    for episode in range(0, config.num_eps):
        world = client.load_world(town_id,reset_settings=True)
        
        ## SYNCHRONOUS MODE
        #settings = world.get_settings()
        #settings.synchronous_mode = True
        #settings.fixed_delta_seconds = 0.01
        #world.apply_settings(settings)
        #traffic_manager = client.get_trafficmanager()
        #traffic_manager.set_synchronous_mode(True)
        #traffic_manager.set_random_device_seed(9) 
        #tm_port = traffic_manager.get_port()

        ## actor location changes each episode
        ##actor_start_locs = sorted(world.get_map().get_spawn_points() )
        #actor_start_locs = sorted([ [worldpt.location.x,worldpt.location.y,worldpt.location.z] for worldpt in world.get_map().get_spawn_points()])
        actor_start_locs = world.get_map().get_spawn_points()
        #np.random.shuffle(actor_start_locs)
        #pdb.set_trace()
        ## actor information changes each episode
        num_nonego_vehicles = config.num_vehicles
        blueprint_library = world.get_blueprint_library()   
        vehicle_blueprints = blueprint_library.filter('vehicle.*')
        safe_vehicle_blueprints = [x for x in vehicle_blueprints if not x.id.endswith('isetta')]
        np.random.shuffle(safe_vehicle_blueprints) 

        ## run this episode/trajectory for each sun location
        for ii,eps_sunloc in enumerate(sun_locs):

            output_dir = os.path.join(config.output_dir,town_id, 'episode'+str(episode), 'sunloc_'+','.join([str(sl) for sl in eps_sunloc]))
            os.makedirs(output_dir, exist_ok=True)
            os.makedirs(os.path.join(output_dir,'Lidar'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'IMU'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/RGB'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/semantic_segmentation'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Depth'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Normals'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Basecolor'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Specular'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Roughness'), exist_ok=True)
            #os.makedirs(os.path.join(output_dir,'RightCam/Metallic'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/RGB'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/semantic_segmentation'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Depth'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Normals'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Basecolor'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Specular'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Roughness'), exist_ok=True)
            os.makedirs(os.path.join(output_dir,'LeftCam/Metallic'), exist_ok=True) 

            #world = client.reload_world(reset_settings=True)
            world = client.load_world(town_id,reset_settings=True)
            
            ## SYNCHRONOUS MODE FOR RELOADING WORLD
            #settings = world.get_settings()
            #settings.synchronous_mode = True
            #settings.fixed_delta_seconds = 0.05
            #world.apply_settings(settings)
            #traffic_manager = client.get_trafficmanager()
            #traffic_manager.set_synchronous_mode(True)
            #traffic_manager.set_random_device_seed(29) 
            #tm_port = traffic_manager.get_port()

            ## actor location changes each episode

            ##actor_start_locs = sorted(world.get_map().get_spawn_points() )
            #actor_start_locs = sorted([ [worldpt.location.x,worldpt.location.y,worldpt.location.z] for worldpt in world.get_map().get_spawn_points()])
            #np.random.shuffle(actor_start_locs)

            ## actor information changes each episode
            #num_nonego_vehicles = config.num_vehicles
            #blueprint_library = world.get_blueprint_library()   
            #vehicle_blueprints = blueprint_library.filter('vehicle.*')
            #safe_vehicle_blueprints = [x for x in vehicle_blueprints if not x.id.endswith('isetta')]
            #np.random.shuffle(safe_vehicle_blueprints) 

            actor_list =[]
            eps_az = eps_sunloc[0]
            eps_alt = eps_sunloc[1] 
            ## Once we have a client we can retrieve the world that is currently running.
            #world = client.get_world() 
            #
            ###########################
            ## SET UP THE GAME WORLD ##
            ###########################
            ## get weather
            #weather = Weather(world.get_weather())
            weather = carla.WeatherParameters(sun_azimuth_angle=eps_az, sun_altitude_angle=eps_alt)
            ## set the weather for the world
            world.set_weather(weather)
            #
            ## The world contains the list blueprints that we can use for adding new actors into the simulation.
            #num_nonego_vehicles = 20#config.num_vehicles
            #blueprint_library = world.get_blueprint_library()   
            #vehicle_blueprints = blueprint_library.filter('vehicle.*')
            #safe_vehicle_blueprints = [x for x in vehicle_blueprints if not x.id.endswith('isetta')]
            ego_vehicle_blueprint = safe_vehicle_blueprints[0]
            #nonego_vehicle_blueprints = safe_vehicle_blueprints[1:]
            #car_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4 and not x.id.endswith('isetta')]
            #bike_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 2]
            #pedestrian_blueprints = blueprint_library.filter('walker.*')
            ## get the vehicles in the world
            #vehicles = world.get_actors().filter('vehicle.*')
            #
            ## Now we need to give an initial transform to the vehicle. We choose a
            ## random transform from the list of recommended spawn points of the map.
            ##transform = random.choice(world.get_map().get_spawn_points())
 
            ego_start_loc = actor_start_locs[0]
            #nonego_start_locs = actor_start_locs[1:] 
            # 
            ############################
            ## SET UP THE EGO VEHICLE ##
            ############################
             
            ## So let's tell the world to spawn the vehicle.

            #ego_vehicle = world.spawn_actor(ego_vehicle_blueprint, carla.Transform(carla.Location(x=ego_start_loc[0], y=ego_start_loc[1], z=ego_start_loc[2])))  
            ego_vehicle = world.spawn_actor(ego_vehicle_blueprint, ego_start_loc)
            actor_list.append(ego_vehicle)
            print('created %s' % ego_vehicle.type_id)   
            ## Let's put the vehicle to drive around.
            #time.sleep(1)
            ego_vehicle.set_autopilot(True) 
            #
            ################################################
            ## ADD CAMERAS/DATA COLLECTION TO EGO VEHICLE ##
            ################################################
            
            imu_bp = world.get_blueprint_library().find('sensor.other.imu')
            imu_location = carla.Location(0,0,0)
            imu_rotation = carla.Rotation(0,0,0)
            imu_transform = carla.Transform(imu_location,imu_rotation)
            #imu_bp.set_attribute("sensor_tick",str(3.0))
            ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
            def imu_callback(imu):
                #print("IMU measure:\n"+str(imu)+'\n')
                imu_dict={}
                imu_dict['frame'] = imu.frame
                imu_dict['timestamp'] = imu.timestamp
                imu_dict['transform'] = ','.join([str(j) for j in [imu.transform.location.x, imu.transform.location.y, imu.transform.location.z, imu.transform.rotation.pitch, imu.transform.rotation.yaw, imu.transform.rotation.roll] ])
                imu_dict['accelerometer'] = ','.join([str(j) for j in [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]])
                imu_dict['gyroscope']  = ','.join([str(j) for j in [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]])
                imu_dict['compass'] = imu.compass
                #csv_columns = ['frame','timestamp','transform', 'accelerometer', 'gyroscope', 'compass']
                csv_file = os.path.join(output_dir, "IMU/%.6d.txt"%imu.frame)
                with open(csv_file, 'w') as ff:
                    #f = csv.writer(ff)
                    #for key_ in imu_dict.keys():
                    #    f.writerow("%s,%s\n"%(key_,imu_dict[key_]))
                    ff.write('\n'.join(["%s"%(imu_dict[key_]) for key_ in imu_dict]))
            ego_imu.listen(lambda imu: imu_callback(imu))

            # lidar ## 
            lidar_cam = None
            lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels',str(32))
            lidar_bp.set_attribute('points_per_second',str(90000))
            lidar_bp.set_attribute('rotation_frequency',str(40))
            lidar_bp.set_attribute('range',str(20))
            lidar_location = carla.Location(0,0,2.5)
            lidar_rotation = carla.Rotation(0,0,0)
            lidar_transform = carla.Transform(lidar_location,lidar_rotation)
            lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
            actor_list.append(lidar_sen)
            lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk(output_dir+'/Lidar/%.6d.ply' % point_cloud.frame))

#            ## stereo right camera ##
#            camera_transform_R = carla.Transform(carla.Location(x=1.5, z=2.4, y=-0.15))
#            #
#            ## Let's add now a "rgb" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
#            camera_bp_rgb_R = blueprint_library.find('sensor.camera.rgb')
#            camera_rgb_R = world.spawn_actor(camera_bp_rgb_R, camera_transform_R, attach_to=ego_vehicle)
#            actor_list.append(camera_rgb_R)
#            print('created %s' % camera_rgb_R.type_id) 
#            camera_rgb_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/RGB/%06d_rgbr.png'% image.frame ))
#            #
#            ## Let's add now a "semantic_segmentation" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
#            camera_bp_semantic_segmentation_R = blueprint_library.find('sensor.camera.semantic_segmentation')
#            camera_semantic_segmentation_R = world.spawn_actor(camera_bp_semantic_segmentation_R, camera_transform_R, attach_to=ego_vehicle)
#            actor_list.append(camera_semantic_segmentation_R)
#            print('created %s' % camera_semantic_segmentation_R.type_id) 
#            camera_semantic_segmentation_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
#            #
#            ## Let's add now a "depth" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
#            camera_bp_depth_R = blueprint_library.find('sensor.camera.depth')
#            camera_depth_R = world.spawn_actor(camera_bp_depth_R, camera_transform_R, attach_to=ego_vehicle)
#            actor_list.append(camera_depth_R)
#            print('created %s' % camera_depth_R.type_id)    
#            ## Now we register the function that will be called each time the sensor receives an image. In this example we are saving the image to disk converting the pixels to gray-scale.
#            ccdep = carla.ColorConverter.LogarithmicDepth
#            #camera_depth_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Depth/%s_%06d_%s.png'%(str(episode), image.frame, str(eps_sunloc)), ccdep))   
#            camera_depth_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Depth/%06d_depth.png'%image.frame, ccdep))   
#            #
#            ## add normal map camera
#            normalcam_bp_R = blueprint_library.find('sensor.camera.normal_map')
#            normalcam_R = world.spawn_actor(normalcam_bp_R, camera_transform_R , attach_to=ego_vehicle)
#            actor_list.append(normalcam_R)
#            print('created %s'%normalcam_R.type_id)
#            #normalcam.listen(lambda image: image.save_to_disk('/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/carla_out/%06d_normal.png' % image.frame))  
#            normalcam_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Normals/%06d_normal.png' % image.frame))  #

#            ## add basecolor map camera
#            bccam_bp_R = blueprint_library.find('sensor.camera.mat_map')
#            bccam_R = world.spawn_actor(bccam_bp_R, camera_transform_R , attach_to=ego_vehicle)
#            actor_list.append(bccam_R)
#            print('created %s'%bccam_R.type_id)
#            bccam_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #

#            ## add specular map camera
#            speccam_bp_R = blueprint_library.find('sensor.camera.specular')
#            speccam_R = world.spawn_actor(speccam_bp_R, camera_transform_R , attach_to=ego_vehicle)
#            actor_list.append(speccam_R)
#            print('created %s'%speccam_R.type_id)
#            speccam_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Specular/%06d_specularfromlighting.png' % image.frame))  #

#            ## add roughness map camera
#            roughcam_bp_R = blueprint_library.find('sensor.camera.roughness')
#            roughcam_R = world.spawn_actor(roughcam_bp_R, camera_transform_R , attach_to=ego_vehicle)
#            actor_list.append(roughcam_R)
#            print('created %s'%roughcam_R.type_id)
#            roughcam_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Roughness/%06d_Roughness.png' % image.frame))    #

#            ## add metallic map camera
#            metcam_bp_R = blueprint_library.find('sensor.camera.metallic')
#            metcam_R = world.spawn_actor(metcam_bp_R, camera_transform_R , attach_to=ego_vehicle)
#            actor_list.append(metcam_R)
#            print('created %s'%metcam_R.type_id)
#            metcam_R.listen(lambda image: image.save_to_disk(output_dir+'/RightCam/Metallic/%06d_metallic.png' % image.frame))

            ## stereo left camera ##
            #camera_transform_L = carla.Transform(carla.Location(x=1.5, z=2.4, y=0.15))#
            camera_transform_L = carla.Transform(carla.Location(x=1.5, z=2.4))#
            ## Let's add now a "rgb" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_Lgb_L = blueprint_library.find('sensor.camera.rgb')
            camera_Lgb_L = world.spawn_actor(camera_bp_Lgb_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_Lgb_L)
            print('created %s' % camera_Lgb_L.type_id) 
            #camera_Lgb_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/RGB/%s_%06d_%s.png'%(str(episode), image.frame, str(eps_sunloc))))
            camera_Lgb_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/RGB/%06d_rgbl.png'%image.frame))
            #
            ## Let's add now a "semantic_segmentation" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_semantic_segmentation_L = blueprint_library.find('sensor.camera.semantic_segmentation')
            camera_semantic_segmentation_L = world.spawn_actor(camera_bp_semantic_segmentation_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_semantic_segmentation_L)
            print('created %s' % camera_semantic_segmentation_L.type_id) 
            camera_semantic_segmentation_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/semantic_segmentation/%06d_cityscapes.png'%image.frame, cc.CityScapesPalette))
            #
            ## Let's add now a "depth" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
            camera_bp_depth_L = blueprint_library.find('sensor.camera.depth')
            camera_depth_L = world.spawn_actor(camera_bp_depth_L, camera_transform_L, attach_to=ego_vehicle)
            actor_list.append(camera_depth_L)
            print('created %s' % camera_depth_L.type_id)    
            ## Now we register the function that will be called each time the sensor receives an image. In this example we are saving the image to disk converting the pixels to gray-scale.
            #ccdep = carla.ColorConverter.LogarithmicDepth
            #camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Depth/%06d_depth.png'%image.frame, ccdep))   
            camera_depth_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Depth/%06d_depth.png'%image.frame))   
            #
            ## add normal map camera
            normalcam_bp_L = blueprint_library.find('sensor.camera.normal_map')
            normalcam_L = world.spawn_actor(normalcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(normalcam_L)
            print('created %s'%normalcam_L.type_id)
            #normalcam.listen(lambda image: image.save_to_disk('/mnt/workspace/users/askc/ILLUMINATION-TRANSFER-PTCLOUD-RGB/datasets/carla_out/%06d_normal.png' % image.frame))  
            normalcam_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Normals/%06d_normal.png' % image.frame))  #
            ## add basecolor map camera
            bccam_bp_L = blueprint_library.find('sensor.camera.mat_map')
            bccam_L = world.spawn_actor(bccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(bccam_L)
            print('created %s'%bccam_L.type_id)
            bccam_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Basecolor/%06d_basecolorfromlighting.png' % image.frame))   #
            ## add specular map camera
            speccam_bp_L = blueprint_library.find('sensor.camera.specular')
            speccam_L = world.spawn_actor(speccam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(speccam_L)
            print('created %s'%speccam_L.type_id)
            speccam_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Specular/%06d_specularfromlighting.png' % image.frame))  #
            ## add roughness map camera
            roughcam_bp_L = blueprint_library.find('sensor.camera.roughness')
            roughcam_L = world.spawn_actor(roughcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(roughcam_L)
            print('created %s'%roughcam_L.type_id)
            roughcam_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Roughness/%06d_roughness.png' % image.frame))    #
            ## add metallic map camera
            metcam_bp_L = blueprint_library.find('sensor.camera.metallic')
            metcam_L = world.spawn_actor(metcam_bp_L, camera_transform_L , attach_to=ego_vehicle)
            actor_list.append(metcam_L)
            print('created %s'%metcam_L.type_id)
            metcam_L.listen(lambda image: image.save_to_disk(output_dir+'/LeftCam/Metallic/%06d_metallic.png' % image.frame))#


            ###########################
            ## ADD IN NON EGO ACTORS ##
            ###########################
            #
            ##resetind = min( (len(nonego_vehicle_blueprints), len(nonego_start_locs)) )
            #for carind in range(1, num_nonego_vehicles):
            #    if carind>len(nonego_vehicle_blueprints)-1:
            #        bpind = carind - len(nonego_vehicle_blueprints) -1
            #    else:
            #        bpind = carind
            #    temp_vehicle = nonego_vehicle_blueprints[bpind]
            #    #
            #    if carind>len(nonego_start_locs)-1:
            #        locind = carind - len(nonego_start_locs) -1
            #    else:
            #        locind = carind
            #    temp_vehicle_strt = nonego_start_locs[locind]
            #    ## spawn the actor
            #    #npc = world.spawn_actor(temp_vehicle, temp_vehicle_strt)
            #    npc = world.try_spawn_actor(temp_vehicle, carla.Transform(carla.Location(x=temp_vehicle_strt[0], y=temp_vehicle_strt[1], z=temp_vehicle_strt[2])))
            #    if npc is not None:
            #        actor_list.append(npc)
            #        #time.sleep(1)
            #        npc.set_autopilot(True, tm_port)
            #        #pdb.set_trace()
            #        print('created %s at (%f,%f,%f)' %(npc.type_id,temp_vehicle_strt[0],temp_vehicle_strt[1],temp_vehicle_strt[2]))   
            #        ##
            #pdb.set_trace()
            #
            #world.tick()
            #agent = BasicAgent(ego_vehicle)
            #agent.set_destination(carla.Location(x=temp_vehicle_strt[0], y=temp_vehicle_strt[1], z=temp_vehicle_strt[2]))

            #sync_mode = CarlaSyncMode(world, camera_rgb_R, camera_rgb_L, 
            #                                 camera_semantic_segmentation_R, camera_semantic_segmentation_L, 
            #                                 camera_depth_R, camera_depth_L, 
            #                                 normalcam_R, normalcam_L,
            #                                 roughcam_R, roughcam_L,
            #                                 bccam_R, bccam_L,
            #                                 speccam_R, speccam_L,
            #                                 metcam_R, metcam_L,
            #                                 fps=30) 
            ########################
            ## RUN THE SIMULATION ##
            ########################
            ## run the episode for the specified number of frames
            elapsed_time = 0.0
            while True:

                world.tick()
                
                #control = agent.run_step()
                #control.manual_gear_shift = False
                #ego_vehicle.apply_control(control)

                elapsed_time += 1#timestamp.delta_seconds

                if elapsed_time > config.total_time:
                    ## destroy actors
                    print('destroying actors')
                    #pdb.set_trace()
                    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
                    print('done.')
                    break
            time.sleep(0.5) 

#########################################################################
## MAIN FUNCTION
#########################################################################

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--num_eps',
        metavar='number_of_episodes',
        default=1,
        help='number of episodes to run, determines the number of sun angles as well')
    argparser.add_argument(
        '--num_frames',
        metavar='number_frames_per_episode',
        default=250,
        type = int,
        help='number of frames to collect per episode')
    argparser.add_argument(
        '--num_vehicles',
        metavar='number_frames_per_episode',
        default=10,
        type = int,
        help='number of frames to collect per episode')    
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Roaming", "Basic"],
                           help="select which agent to run",
                           default="Basic")
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    argparser.add_argument(
        '-od', '--output_dir',
        metavar='output_dir',
        default='output',
        type=str,
        help='location where to save the output images from the sensors')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]
    args.num_eps = int(args.num_eps)
    args.total_time = float(args.num_frames)#/10.0 ## assuming carlaUE4 is run with 10fps

    ## run game
    town_ids = ['Town01_Opt','Town02_Opt','Town03_Opt','Town04_Opt','Town05_Opt','Town06_Opt','Town07_Opt','Town08_Opt','Town09_Opt'] 
    #town_ids = ['Town04_Opt','Town05_Opt','Town06_Opt','Town07_Opt','Town08_Opt','Town09_Opt'] 
    for town_id in town_ids:
        run_game(args, town_id)

if __name__ == '__main__':

    main()
