'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform

class HidingSpot(object):
    # Obstacle Setup ---------------------------------------------------------------
    def __init__(self, pos=Vector2D(0,0), d_to_evader=9999999, d_to_hunter=9999999999):
        # where am i? 
        self.pos = pos
        self.dist_to_evader = d_to_evader
        self.dist_to_hunter = d_to_hunter

        # how good the hiding spot is; the lower the rank no. the better
        self.rank = 0.0
        self.collisions = 0

    # The central logic of the Obstacle class ------------------------------------------------

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist
