'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform
from hiding_spot import HidingSpot

class Obstacle(object):
    # Obstacle Setup ---------------------------------------------------------------
    def __init__(self, world=None, scale=30.0):
        # keep a reference to the world object
        self.world = world

        # scaling variables
        self.scale_scalar = scale
        #self.scale_vector = Vector2D(scale, scale)  # easy scaling of agent size

        #self.b_radius = self.scale_scalar
        self.hiding_spot = HidingSpot(self.world)

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

        # where am i? random
        self.pos = self.get_random_valid_position(self.world.cx, self.world.cy, self.world.obstacles, self.world.agents)

    # The central logic of the Obstacle class ------------------------------------------------

    def check_position_valid(self):
        if self.pos.x < 0 or self.pos.x > self.world.cx or self.pos.y < 0 or self.pos.y > self.world.cy:
            self.randomise_position()

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

    def get_random_valid_position(self, max_x, max_y, obstacles, agents):
        valid = False
        pos = Vector2D()

        while not valid:
            valid = True
            pos = Vector2D(randrange(max_x), randrange(max_y))
            
            for obstacle in obstacles:
                if obstacle is not self and obstacle.distance(pos) <= obstacle.radius * 1.5:
                    valid = False

            for agent in agents:
                if agent.distance(pos) <= self.radius + agent.avoid_radius:
                    valid = False

        return pos

    def randomise_position(self):
        self.pos = self.get_random_valid_position(self.world.cx, self.world.cy, self.world.obstacles, self.world.agents)
        self.hiding_spot.collisions = 0

    def render(self, color=None):
        # draw the obstacle according to its default colour
        egi.set_pen_color(name=self.color)
        egi.set_stroke(2)
        egi.circle(self.pos, self.radius)

        self.hiding_spot.render()