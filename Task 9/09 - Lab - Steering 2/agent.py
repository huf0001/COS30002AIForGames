'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange

AGENT_MODES = {
    KEY._1: 'seek',
    KEY._2: 'arrive_slow',
    KEY._3: 'arrive_normal',
    KEY._4: 'arrive_fast',
    KEY._5: 'flee',
    KEY._6: 'pursuit',
    KEY._7: 'follow_path',
    KEY._8: 'wander',
}

class Agent(object):

    # NOTE: Class Object (not *instance*) variables!
    DECELERATION_SPEEDS = {
        'slow': 0.9,
        'normal': 0.6,
        'fast': 0.3
    }

    def __init__(self, world=None, scale=30.0, mass=1.0, mode=None):
        # keep a reference to the world object
        self.world = world
        self.mode = mode
        # where am i and where am i going? random
        dir = radians(random()*360)
        self.pos = Vector2D(randrange(world.cx), randrange(world.cy))
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.scale = Vector2D(scale, scale)  # easy scaling of agent size
        self.force = Vector2D()  # current steering force
        self.accel = Vector2D() # current acceleration due to force
        self.applying_friction = False


        AGENT_MODELS = [
        	'dart',
        	'block',
        	'ufo'
        ]

        model = AGENT_MODELS[randrange(0, 3)]

        if model == "dart":
            self.mass = 1.0
            # limits?
            self.max_forward_speed = 5000.0# * self.scale.length()
            self.max_sideways_speed = 4000.0# * self.scale.length()
            self.max_reverse_speed = 1000.0# * self.scale.length()
            self.friction = 0.1
            # data for drawing this agent
            self.color = 'ORANGE'
            self.vehicle_shape = [
                Point2D(-1.0,  0.6),
                Point2D( 1.0,  0.0),
                Point2D(-1.0, -0.6)
            ]
        elif model == "block":
            self.mass = 1.5
            # limits?
            self.max_forward_speed = 4000.0# * self.scale.length()
            self.max_sideways_speed = 3200.0# * self.scale.length()
            self.max_reverse_speed = 1000.0# * self.scale.length()
            self.friction = 0.2
            # data for drawing this agent
            self.color = 'RED'
            self.vehicle_shape = [
                Point2D(-1.0,  0.6),
                Point2D( 1.0,  0.6),
                Point2D( 1.0, -0.6),
                Point2D(-1.0, -0.6)
            ]
        else:
            self.mass = 2.0
            # limits?
            self.max_forward_speed = 3000.0# * self.scale.length()
            self.max_sideways_speed = 2400.0#* self.scale.length()
            self.max_reverse_speed = 1000.0# * self.scale.length()
            self.friction = 0.3
            # data for drawing this agent
            self.color = 'PURPLE'
            self.vehicle_shape = [
                Point2D( 0.4,  1.0),
                Point2D(-0.4,  1.0),
                Point2D(-1.0,  0.4),
                Point2D(-1.0, -0.4),
                Point2D(-0.4, -1.0),
                Point2D( 0.4, -1.0),
                Point2D( 1.0, -0.4),
                Point2D( 1.0,  0.4)
            ]

        ### path to follow?
        # self.path = ??

        ### wander details
        # self.wander_?? ...

        # limits?
        #self.max_speed = 20.0 * scale
        ## max_force ??

        # debug draw info?
        self.show_info = False

    def calculate(self):
        # reset the steering force
        mode = self.mode
        if mode == 'seek':
            force = self.seek(self.world.target)
        elif mode == 'arrive_slow':
            force = self.arrive(self.world.target, 'slow')
        elif mode == 'arrive_normal':
            force = self.arrive(self.world.target, 'normal')
        elif mode == 'arrive_fast':
            force = self.arrive(self.world.target, 'fast')
        elif mode == 'flee':
            force = self.flee(self.world.target)
        elif mode == 'pursuit':
            force = self.pursuit(self.world.evader)
        elif mode == 'wander':
            force = self.wander(delta)
        elif mode == 'follow_path':
            force = self.follow_path()
        else:
            force = Vector2D()
        self.force = force
        return force

    def update(self, delta):
        ''' update vehicle position and orientation '''
        force = self.calculate()
         ## limit force? <-- for wander
        # ...
        # determin the new accelteration
        self.accel = force * (1 / self.mass)  # not needed if mass = 1.0
        # new velocity
        self.vel += self.accel * delta
        # check for limits of new velocity
        #self.vel.truncate(self.max_forward_speed)
        self.vel = self.enforce_speed_limit(self.vel)
        # apply friction
        if self.applying_friction:
        	self.vel += self.apply_friction() * delta
        # update position
        self.pos += self.vel * delta
        # update heading is non-zero velocity (moving)
        if self.vel.lengthSq() > 0.00000001:
            self.heading = self.vel.get_normalised()
            self.side = self.heading.perp()
        # treat world as continuous space - wrap new position if needed
        self.world.wrap_around(self.pos)

    def render(self, color=None):
        # draw the path if it exists and the mode is follow
        if self.mode == 'follow_path':
            ## ...
            pass

        # draw the ship
        if self.world.agent_mode == 'pursuit':
        	if self.mode == 'pursuit':
        		egi.set_pen_color(name='RED')
        	else:
        		egi.set_pen_color(name='WHITE')
        else:
        	egi.set_pen_color(name=self.color)
        egi.set_stroke(2)
        pts = self.world.transform_points(self.vehicle_shape, self.pos,
                                          self.heading, self.side, self.scale)
        # draw it!
        egi.closed_shape(pts)

        # draw wander info?
        if self.mode == 'wander':
            ## ...
            pass

        # add some handy debug drawing info lines - force and velocity
        if self.show_info:
            s = 0.5 # <-- scaling factor
            # force
            egi.red_pen()
            egi.line_with_arrow(self.pos, self.pos + self.force * s, 5)
            # velocity
            egi.grey_pen()
            egi.line_with_arrow(self.pos, self.pos + self.vel * s, 5)
            # net (desired) change
            egi.white_pen()
            egi.line_with_arrow(self.pos+self.vel * s, self.pos+ (self.force+self.vel) * s, 5)
            egi.line_with_arrow(self.pos, self.pos+ (self.force+self.vel) * s, 5)

    def speed(self):
        return self.vel.length()

    def apply_friction(self):
    	future_pos = self.pos + self.vel * 0.1
    	accel_to_future_pos = self.seek(future_pos)
    	friction = accel_to_future_pos * -self.friction
    	return friction
    #--------------------------------------------------------------------------

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_forward_speed
        return (desired_vel - self.vel)

    def flee(self, hunter_pos):
        ''' move away from hunter position '''
        panic_range = 100

        if self.distance(hunter_pos) > panic_range:
        	return Vector2D(0, 0)

        desired_vel = (self.pos - hunter_pos).normalise() * self.max_forward_speed
        return (desired_vel - self.vel)

    def arrive(self, target_pos, speed):
        ''' this behaviour is similar to seek() but it attempts to arrive at
            the target position with a zero velocity'''
        decel_rate = self.DECELERATION_SPEEDS[speed]
        to_target = target_pos - self.pos
        dist = to_target.length()
        if dist > 0:
            # calculate the speed required to reach the target given the
            # desired deceleration rate
            speed = dist / decel_rate
            # make sure the velocity does not exceed the max
            speed = min(speed, self.max_forward_speed)
            # from here proceed just like Seek except we don't need to
            # normalize the to_target vector because we have already gone to the
            # trouble of calculating its length for dist.
            desired_vel = to_target * (speed / dist)
            return (desired_vel - self.vel)
        return Vector2D(0, 0)

    def pursuit(self, evader):
    	''' this behaviour predicts where an agent will be in time T and seeks
    		towards that point to intercept it. '''
    	to_evader = evader.pos - self.pos
    	relative_heading = self.heading.dot(evader.heading)

    	if (to_evader.dot(self.heading) > 0) and (relative_heading < 0.95):
    		return self.seek(evader.pos)

    	future_time = to_evader.length()/(self.max_forward_speed + evader.speed())
    	#future_time += (1 - self.heading.dot(evader.vel))
    	future_pos = evader.pos + evader.vel * future_time
    	return self.seek(future_pos)

    def wander(self, delta):
        ''' Random wandering using a projected jitter circle. '''
        ## ...
        return Vector2D()

    def distance(self, target_pos):
    	to_target = target_pos - self.pos
    	dist = to_target.length()
    	return dist

    def enforce_speed_limit(self, vel):
    	result = vel

    	if result.x >= 0:
    		result.x = min(result.x, self.max_forward_speed)
    	else:
    		result.x = max(result.x, -self.max_reverse_speed)	# max_reverse_speed is asigned as +ve
    															# when applying, it needs to be -ve
    	if result.y >= 0:	
    		result.y = min(result.y, self.max_sideways_speed)
    	else:
    		result.y = max(result.y, -self.max_sideways_speed)

    	return result
