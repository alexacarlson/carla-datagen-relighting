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
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

import random
import time

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


def run_game(config):
    actor_list = []

    # First of all, we need to create the client that will send the requests
    # to the simulator. Here we'll assume the simulator is accepting
    # requests in the localhost at port 2000.
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    #pdb.set_trace()

    speed_factor = 1.0#config.speed
    update_freq = 0.1 / speed_factor

    for episode in range(0, config.num_eps):
        #
        # Once we have a client we can retrieve the world that is currently running.
        world = client.get_world() 
        #
        ###########################
        ## SET UP THE GAME WORLD ##
        ###########################
        ## get weather
        weather = Weather(world.get_weather())
        #weather = carla.WeatherParameters(cloudyness=80.0,
        #                                precipitation=30.0,
        #                                sun_azimuth_angle=70.0,
        #                                sun_altitude_angle=70.0)
        ### set the weather for the world
        #world.set_weather(weather)
        #
        # The world contains the list blueprints that we can use for adding new actors into the simulation.
        num_nonego_vehicles = 20#config.num_vehicles
        blueprint_library = world.get_blueprint_library()   
        vehicle_blueprints = blueprint_library.filter('vehicle.*')
        safe_vehicle_blueprints = [x for x in vehicle_blueprints if not x.id.endswith('isetta')]
        ego_vehicle_blueprint = safe_vehicle_blueprints[0]
        nonego_vehicle_blueprints = safe_vehicle_blueprints[1:]
        #car_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4 and not x.id.endswith('isetta')]
        #bike_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 2]
        #pedestrian_blueprints = blueprint_library.filter('walker.*')
        ## get the vehicles in the world
        #vehicles = world.get_actors().filter('vehicle.*')
        #
        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        #transform = random.choice(world.get_map().get_spawn_points())
        actor_start_locs = world.get_map().get_spawn_points()  
        ego_start_loc = actor_start_locs[0]
        nonego_start_locs = actor_start_locs[1:] 
        # 
        #pdb.set_trace()
        ############################
        ## SET UP THE EGO VEHICLE ##
        ############################
        # So let's tell the world to spawn the vehicle.
        ego_vehicle = world.spawn_actor(ego_vehicle_blueprint, ego_start_loc)  
        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(ego_vehicle)
        print('created %s' % ego_vehicle.type_id)   
        # Let's put the vehicle to drive around.
        ego_vehicle.set_autopilot(True) 
        #
        ################################################
        ## ADD CAMERAS/DATA COLLECTION TO EGO VEHICLE ##
        ################################################
        # Let's add now a "rgb" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
        camera_bp_rgb = blueprint_library.find('sensor.camera.rgb')
        camera_transform_rgb = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera_rgb = world.spawn_actor(camera_bp_rgb, camera_transform_rgb, attach_to=ego_vehicle)
        actor_list.append(camera_rgb)
        print('created %s' % camera_rgb.type_id) 
        camera_rgb.listen(lambda image: image.save_to_disk(config.output_dir+'/RGB/%06d.png' % image.frame_number))
        #
        # Let's add now a "semantic_segmentation" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
        camera_bp_semantic_segmentation = blueprint_library.find('sensor.camera.semantic_segmentation')
        camera_transform_semantic_segmentation = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera_semantic_segmentation = world.spawn_actor(camera_bp_semantic_segmentation, camera_transform_semantic_segmentation, attach_to=ego_vehicle)
        actor_list.append(camera_semantic_segmentation)
        print('created %s' % camera_semantic_segmentation.type_id) 
        camera_semantic_segmentation.listen(lambda image: image.save_to_disk(config.output_dir+'/semantic_segmentation/%06d.png' % image.frame_number, cc.CityScapesPalette))
        #
        # Let's add now a "depth" camera attached to the vehicle. Note that the transform we give here is now relative to the vehicle.
        camera_bp_depth = blueprint_library.find('sensor.camera.depth')
        camera_transform_depth = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera_depth = world.spawn_actor(camera_bp_depth, camera_transform_depth, attach_to=ego_vehicle)
        actor_list.append(camera_depth)
        print('created %s' % camera_depth.type_id)    
        # Now we register the function that will be called each time the sensor receives an image. In this example we are saving the image to disk converting the pixels to gray-scale.
        ccdep = carla.ColorConverter.LogarithmicDepth
        camera_depth.listen(lambda image: image.save_to_disk(config.output_dir+'/Depth/%06d.png' % image.frame_number, ccdep))   
        #
        ###########################
        ## ADD IN NON EGO ACTORS ##
        ###########################
        #
        #resetind = min( (len(nonego_vehicle_blueprints), len(nonego_start_locs)) )
        for carind in range(1, num_nonego_vehicles):
            if carind>len(nonego_vehicle_blueprints)-1:
                bpind = carind - len(nonego_vehicle_blueprints) -1
            else:
                bpind = carind
            temp_vehicle = nonego_vehicle_blueprints[bpind]
            #
            if carind>len(nonego_start_locs)-1:
                locind = carind - len(nonego_start_locs) -1
            else:
                locind = carind
            temp_vehicle_strt = nonego_start_locs[locind]
            ## spawn the actor
            #npc = world.spawn_actor(temp_vehicle, temp_vehicle_strt)
            npc = world.try_spawn_actor(temp_vehicle, temp_vehicle_strt)
            if npc is not None:
                actor_list.append(npc)
                npc.set_autopilot()
                #pdb.set_trace()
                print('created %s at (%s,%s,%s)' %(npc.type_id,temp_vehicle_strt.location.x,temp_vehicle_strt.location.y,temp_vehicle_strt.location.z))   
                ##
        #time.sleep(5)   
        #
        ### run the episode for the specified number of frames
        #if carla.Timestamp.frame_count > config.num_frames:
        #    ## destroy actors
        #    print('destroying actors')
        #    for actor in actor_list:
        #        actor.destroy()
        #    print('done.')
        #    break
        #    #carla.SensorData.frame_number
        #
        ### run the episode while varying sun position
        #weather = Weather(world.get_weather())  
        elapsed_time = 0.0  
        while True:
            #pdb.set_trace()
            timestamp = world.wait_for_tick(seconds=30.0)#.timestamp
            elapsed_time += timestamp.delta_seconds
            if elapsed_time > update_freq:
                weather.tick(speed_factor * elapsed_time)
                world.set_weather(weather.weather)
                sys.stdout.write('\r' + str(weather) + 12 * ' ')
                sys.stdout.flush()
                elapsed_time = 0.0
            #pdb.set_trace()
            #if carla.Timestamp.frame_count > config.num_frames:
            #if timestamp.frame_count > config.num_frames:
            #    pdb.set_trace()
            #    ## destroy actors
            #    print('destroying actors')
            #    for actor in actor_list:
            #        actor.destroy()
            #    print('done.')
            #    break
        # destroy actors after simulation is over
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

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
        default=20,
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
    ## run game
    run_game(args)

if __name__ == '__main__':

    main()