'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform

class Wall(object):
    # Obstacle Setup ---------------------------------------------------------------
    def __init__(self, world=None, side='none'):
        # keep a reference to the world object
        self.world = world

        # rendering details
        self.side = side
        self.margin = self.world.wall_margin
        self.color = 'GREY'

        self.pos_a = Vector2D()
        self.pos_b = Vector2D()

        self.set_points()

        # self.pts = [
        #     self.pos_a,
        #     self.pos_b
        # ]

    # The central logic of the Obstacle class ------------------------------------------------

    def distance(self, obj_pos):
        to_target = obj_pos - self.get_pos(obj_pos)
        dist = to_target.length()
        return dist

    def get_pos(self, obj_pos):
        if self.side == 'top':
            return Vector2D(obj_pos.x, self.world.cy - self.margin) 
        elif self.side == 'bottom':
            return Vector2D(obj_pos.x, self.margin) 
        elif self.side == 'left':
            return Vector2D(self.margin, obj_pos.y) 
        elif self.side == 'right':
            return Vector2D(self.world.cx - self.margin, obj_pos.y) 
        else:
            return Vector2D()

    def render(self, color=None):
        # draw the obstacle according to its default colour
        egi.set_pen_color(name=self.color)
        egi.set_stroke(2)
        egi.line_by_pos(self.pos_a, self.pos_b)
        # egi.cross(self.pos_a, 5)
        # egi.cross(self.pos_b, 5)

    def set_points(self):
        if self.side == 'top':
            self.pos_a = Vector2D(self.margin, self.world.cy - self.margin) 
            self.pos_b = Vector2D(self.world.cx - self.margin, self.world.cy - self.margin) 
        elif self.side == 'bottom':
            self.pos_a = Vector2D(self.margin, self.margin) 
            self.pos_b = Vector2D(self.world.cx - self.margin, self.margin) 
        elif self.side == 'left':
            self.pos_a = Vector2D(self.margin, self.margin) 
            self.pos_b = Vector2D(self.margin, self.world.cy - self.margin) 
        elif self.side == 'right':
            self.pos_a = Vector2D(self.world.cx - self.margin, self.margin) 
            self.pos_b = Vector2D(self.world.cx - self.margin, self.world.cy - self.margin)
