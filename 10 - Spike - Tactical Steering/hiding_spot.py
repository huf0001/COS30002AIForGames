'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform
from graphics import egi

class HidingSpot(object):
    # Obstacle Setup ---------------------------------------------------------------
    def __init__(self, world):
        # where am i? 
        self.pos = Vector2D()
        # self.dist_to_evader = d_evader
        self.avg_dist_to_hunter = 999999999
        self.dist_to_evader = 0
        self.world = world

        # how good the hiding spot is; the lower the rank no. the better
        self.rank = 0.0
        # self.collisions = 0

    # The central logic of the Obstacle class ------------------------------------------------

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

    def render(self):
        if self.valid:
            egi.red_pen()
            
            for evader in self.world.evaders:
                if self is evader.best_hiding_spot:
                    egi.orange_pen()

            egi.cross(self.pos, 10)
                    
