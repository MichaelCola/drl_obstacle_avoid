from __future__ import print_function

import sys
import signal
import argparse
import numpy as np

from scene_loader import THORDiscreteEnvironment
from utils.tools import SimpleImageViewer

action_space=[0,1,2,3]

def step(env,action):

  human_agent_action=action
 

  env.step(human_agent_action)


    # waiting for reset command
    if human_wants_restart:
      # reset agent to random location
      env.reset()
      human_wants_restart = False

    # check collision
    if env.collided:
      #print('Collision occurs.')
      env.collided = False
