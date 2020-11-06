#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a human agent to control the ego vehicle via keyboard
"""

from __future__ import print_function

import json
import math
import random
import signal
import sys
import pygame

import carla

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from human_agent import  HumanAgent
from imitation_agent import ImitationAgent

class HumanInterface(object):

    """
    Class to control a vehicle manually for debugging purposes
    """

    def __init__(self):
        self._width = 800
        self._height = 600
        self._surface = None

        pygame.init()
        pygame.font.init()
        self._clock = pygame.time.Clock()
        self._display = pygame.display.set_mode((self._width, self._height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Human Agent")

    def run_interface(self, input_data):
        """
        Run the GUI
        """
        # process sensor data
        image_center = input_data['Center'][1][:, :, -2::-1]

        # display image
        self._surface = pygame.surfarray.make_surface(image_center.swapaxes(0, 1))
        if self._surface is not None:
            self._display.blit(self._surface, (0, 0))
        pygame.display.flip()

    def quit_interface(self):
        """
        Stops the pygame window
        """
        pygame.quit()






def clean_up():
    global world
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    CarlaDataProvider.cleanup()

    for actor in actor_list:
        carla.command.DestroyActor(actor)

    global work_agent
    work_agent.cleanup()

    work_agent = None
    global a_agent
    a_agent.destroy()
    global hic
    hic.quit_interface()


def signal_handler(signum, frame):
    # world = CarlaDataProvider.get_world()
    clean_up()


if __name__ == '__main__':
    host = '127.0.0.1'
    port = 2000
    width = 1280
    height = 720
    frame_rate = 20.0
    trafficManagerSeed = 0
    trafficManagerPort = 8000
    city_name = 'Town02'
    actor_list = []
    work_agent = None
    a_agent = None
    world = None
    if sys.platform != 'win32':
        signal.signal(signal.SIGHUP, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # set the world and data_provider
        client = carla.Client(host, port)
        client.set_timeout(2.0)

        world = client.load_world(city_name)
        traffic_manager = client.get_trafficmanager(int(trafficManagerPort))

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / frame_rate
        world.apply_settings(settings)

        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(int(trafficManagerSeed))
        CarlaDataProvider.set_client(client)
        CarlaDataProvider.set_world(world)
        CarlaDataProvider.set_traffic_manager_port(int(trafficManagerPort))

        if CarlaDataProvider.is_sync_mode():
            world.tick()
        else:
            world.wait_for_tick()

        # generate ego vehicle
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle'))

        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)

        # set agent
        # a_agent = HumanAgent(path_to_conf_file="")
        a_agent = ImitationAgent(vehicle, city_name, True)
        work_agent = AgentWrapper(a_agent)
        work_agent.setup_sensors(vehicle)

        hic = HumanInterface()


        clock = pygame.time.Clock()
        while True:
            # game loop
            world = CarlaDataProvider.get_world()
            if world:
                v = vehicle.get_velocity()
                a_agent.current_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                print('speed:', a_agent.current_speed)
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:

                GameTime.on_carla_tick(timestamp)
                CarlaDataProvider.on_carla_tick()
                # generate control command
                ego_action = work_agent()
                hic.run_interface(a_agent.current_data)
                if ego_action:
                    vehicle.apply_control(ego_action)
                else:
                    break

                # in sync mode, run tick to send data to the world
                world.tick()

    finally:
        clean_up()
