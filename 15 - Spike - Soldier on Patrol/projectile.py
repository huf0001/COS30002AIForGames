'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians, sqrt
from random import random, randrange, uniform
from datetime import datetime, time, timedelta


class Projectile(object):
    # Agent Setup ---------------------------------------------------------------

    def __init__(self, world=None, scale=30.0, radius=3.0, pos=Vector2D(0,0), vel=Vector2D(0,0), target=None, shooter=None, p_type=None, damage=None):
        # keep a reference to the world object
        self.world = world

        # what and where am I and where am I going?
        self.pos = pos
        self.vel = vel
        self.heading = self.vel.get_normalised()
        self.side = self.heading.perp()
        self.radius = radius
        self.shooter = shooter
        self.p_type = p_type
        self.damage = damage

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(scale, scale)  # easy scaling of projectile size

        self.target = target
        self.target_radius = self.radius * 2

        self.left_barrel = False
        self.exploding = False
        self.explosion_time = None
        self.explosion_multiplier = 1

    # The central logic of the Projectile class ---------------------------------------------------

    def update(self, delta):
        if not self.left_barrel and self.distance(self.shooter.pos) > self.shooter.radius + self.radius:
            self.left_barrel = True

        if not self.exploding:
            ''' update projectile position '''
            self.pos += self.vel * delta

            if self.pos.x < 0 or self.pos.x > self.world.cx or self.pos.y < 0 or self.pos.y > self.world.cy:
                self.world.destroy_projectile(self)

            collided = self.collided()

            if len(collided) > 0:
                if self.p_type == 'Rocket' or self.p_type == 'Hand Grenade':
                    for collision in collided:
                        if collision is not self.target and collision in self.world.agents:
                            collision.hit_time = datetime.now()
                            collision.health -= self.damage

                    self.target = None
                    self.exploding = True
                    self.explosion_time = datetime.now()
                else:
                    for collision in collided:
                        if collision in self.world.agents:
                            collision.hit_time = datetime.now()
                            collision.health -= self.damage
                    
                    self.world.destroy_projectile(self)
        else:
            self.radius += 0.1 * self.scale_scalar * self.explosion_multiplier

            collided = self.collided()

            if len(collided) > 0:
                for collision in collided:
                    if collision in self.world.agents:
                        collision.hit_time = datetime.now()
                        collision.health -= self.damage

            if self.explosion_multiplier == 1 and (datetime.now() - self.explosion_time).total_seconds() > 1:
                self.explosion_multiplier = -5
            elif self.radius <= 0:
                self.world.destroy_projectile(self)

    def render(self, color=None):
        egi.red_pen()
        egi.circle(self.pos, self.radius)

        if self.exploding:
            egi.circle(self.pos, self.radius * 0.8)
            egi.circle(self.pos, self.radius * 0.6)
            egi.circle(self.pos, self.radius * 0.4)
            egi.circle(self.pos, self.radius * 0.2)
        elif self.target is not None:
            egi.circle(self.target, self.target_radius)
            egi.cross(self.target, self.target_radius)

    # Utility Methods -----------------------------------------------------------------------------

    def collided(self):
        collided = []

        if len(self.world.agents) > 0:
            for agent in self.world.agents:
                if (self.left_barrel or agent is not self.shooter) and self.distance(agent.pos) < self.radius + agent.radius:
                    collided.append(agent)

        if self.world.walls_enabled:
            for wall in self.world.walls:
                if self.distance(wall.get_pos(self.pos)) < self.radius:
                    collided.append(True)
        if self.world.obstacles_enabled:
            for obstacle in self.world.obstacles:
                if self.distance(obstacle.pos) < self.radius + obstacle.radius:
                    collided.append(True)

        if self.target is not None and self.distance(self.target) < self.target_radius:
            collided.append(self.target)

        return collided

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist
