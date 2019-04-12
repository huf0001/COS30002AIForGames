'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform

class Obstacle(object):
    # Obstacle Setup ---------------------------------------------------------------
    def __init__(self, world=None, scale=30.0, mass=1.0):
        # keep a reference to the world object
        self.world = world

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(scale, scale)  # easy scaling of agent size

        # where am i? random
        self.pos = Vector2D(randrange(self.world.cx), randrange(self.world.cy))
        self.b_radius = self.scale_scalar

        OBSTACLE_SIZE = [
            'small',
            'medium',
            'large'
        ]

        size = OBSTACLE_SIZE[randrange(0, 3)]

        if size == "small":
            # data for drawing this obstacle
            self.color = 'ORANGE'
            self.radius = 1 * self.scale_scalar
        elif size == "medium":
            # data for drawing this obstacle
            self.color = 'RED'
            self.radius = 2 * self.scale_scalar
        else:
            # data for drawing this obstacle
            self.color = 'PURPLE'
            self.radius = 3 * self.scale_scalar

    # The central logic of the Obstacle class ------------------------------------------------

    def randomise_position(self):
        self.pos = Vector2D(randrange(self.world.cx), randrange(self.world.cy))

    def render(self, color=None):
        # draw the obstacle according to its default colour
        egi.set_pen_color(name=self.color)
        egi.set_stroke(2)
        egi.circle(self.pos, self.radius)
