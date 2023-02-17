import os
from threading import Thread
from time import sleep

def launchGazebo():
    #creates Thread to run gazebo
    gazebo_thread = Thread(target=babysitGazebo, name="gazebo_thread")
    gazebo_thread.start()

def babysitGazebo():
    #runs gazebo
    WORLD_PATH = '~/osu-uwrt/development/software/src/riptide_gazebo/world2.sdf'
    os.system(f'gz sim {WORLD_PATH}')

    while (True):
        sleep(10)
