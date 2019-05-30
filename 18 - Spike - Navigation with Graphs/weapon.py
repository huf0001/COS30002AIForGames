from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians, sqrt
from random import random, randrange, uniform
from path import Path
from hiding_spot import HidingSpot
from projectile import Projectile
from datetime import datetime, time, timedelta

class Weapon (object):
    def __init__(self, world=None, owner=None, name='', cooldown=1, effective_range=100, damage=25, reload_time=1, magazine_size=8, magazines=3, accuracy_modifier=0, speed=500, stamina_drain=0, damage_factor=1, explosive=False, explosion_radius=0):
        self.world = world
        self.owner = owner
        self.name = name
        self.cooldown = cooldown
        self.effective_range = effective_range
        self.speed = speed
        self.damage = damage
        self.damage_factor = damage_factor
        self.reload_time = reload_time
        self.magazine_size = magazine_size
        self.rounds_left_in_magazine = 0
        self.magazines_left = magazines
        self.accuracy_modifier = accuracy_modifier
        self.stamina_drain = stamina_drain
        self.explosive = explosive
        self.explosion_radius = explosion_radius

        self.projectile_pool = []
        i = 0

        while i < 200:
            self.projectile_pool.append(Projectile(world=self.world, weapon=self))
            i += 1
