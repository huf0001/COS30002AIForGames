'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians, sqrt
from random import random, randrange, uniform
from path import Path

AGENT_MODES = {
    KEY._1: 'seek',
    KEY._2: 'arrive_slow',
    KEY._3: 'arrive_normal',
    KEY._4: 'arrive_fast',
    KEY._5: 'flee',
    KEY._6: 'pursuit',
    KEY._7: 'follow_path',
    KEY._8: 'wander',
    KEY._9: 'hide'
}

class Agent(object):
    # Agent Setup ---------------------------------------------------------------

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
        self.force = Vector2D()  # current steering force
        self.accel = Vector2D() # current acceleration due to force
        self.radius = 1.0 * scale
        self.hiding_spots = []
        self.best_hiding_spot = Point2D()

        # force- and speed-limiting variables
        self.applying_friction = False
        self.max_force = 500.0
        self.avoidance_range = 100.0

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(scale, scale)  # easy scaling of agent size

        # path variables
        self.path = Path()
        self.randomise_path()
        self.waypoint_threshold = 40.0

        # wander variables
        self.wander_dist = 2.0 * self.scale_scalar
        self.wander_radius = 1.0 * self.scale_scalar
        self.wander_jitter = 10.0 * self.scale_scalar
        self.wander_target = Vector2D(1.0, 0)
        self.b_radius = self.scale_scalar
        self.wander_while_fleeing = False

        # debug draw info?
        self.show_info = False

        AGENT_MODELS = [
            'dart',
            'block',
            'ufo'
        ]

        model = AGENT_MODELS[randrange(0, 3)]

        if model == "dart":
            self.mass = 1.0
            # limits?
            self.max_speed = 30.0 * self.scale_scalar
            self.friction = 0.1
            # data for drawing this agent
            #self.color = 'ORANGE'
            self.vehicle_shape = [
                Point2D(-1.0,  0.6),
                Point2D( 1.0,  0.0),
                Point2D(-1.0, -0.6)
            ]
        elif model == "block":
            self.mass = 1.5
            # limits?
            self.max_speed = 22.0 * self.scale_scalar
            self.friction = 0.2
            # data for drawing this agent
            #self.color = 'RED'
            self.vehicle_shape = [
                Point2D(-1.0,  0.6),
                Point2D( 1.0,  0.6),
                Point2D( 1.0, -0.6),
                Point2D(-1.0, -0.6)
            ]
        else:
            self.mass = 2.0
            # limits?
            self.max_speed = 18.0 * self.scale_scalar
            self.friction = 0.3
            # data for drawing this agent
            #self.color = 'PURPLE'
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

    # The central logic of the Agent class ------------------------------------------------

    def calculate(self, delta):
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
        elif mode == 'hide':
            force = self.hide(self.world.hunter, self.world.obstacles, delta)
        elif mode == 'flee':
            force = self.flee(self.world.target, delta)
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
        force = self.calculate(delta)
        #force += self.avoid_obstacles(self.world.obstacles)
        ## limit force
        force.truncate(self.max_force)
        # determine the new accelteration
        self.accel = force * (1 / self.mass)  # not needed if mass = 1.0
        # new velocity
        self.vel += self.accel * delta
        # check for limits of new velocity
        if self.mode == 'wander' or self.wander_while_fleeing:
            self.vel.truncate(self.max_speed / 5)
            self.wander_while_fleeing = False
        else:
            self.vel.truncate(self.max_speed)
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
        # # draw the path if it exists and the mode is follow
        # if self.mode == 'follow_path':
        #     self.path.render()
        
        # # draw the ship according to its status if pursuing
        # elif self.world.agent_mode == 'pursuit':
        # 	if self.mode == 'pursuit':
        # 		egi.set_pen_color(name='RED')
        # 	else:
        # 		egi.set_pen_color(name='WHITE')
        
        # draw wander info if wandering
        #el
        if self.mode == 'wander':
            # calculate the centre of the wander circle in front of the agent
            wnd_pos = Vector2D(self.wander_dist, 0)
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)

            # draw the wander circle
            egi.green_pen()
            egi.circle(wld_pos, self.wander_radius)

            # draw the wander target (little circle on the big circle)
            egi.red_pen()
            wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.circle(wld_pos, 3)

        # draw the ship according to its default colour
        elif self.mode == 'hide':
        	egi.orange_pen()

        egi.set_stroke(2)
        pts = self.world.transform_points(self.vehicle_shape, self.pos, self.heading, self.side, self.scale_vector)
        
        # draw it!
        egi.closed_shape(pts)

        if self.best_hiding_spot:
            egi.orange_pen()
            egi.cross(self.best_hiding_spot, 10)

        egi.red_pen()
        for hiding_spot in self.hiding_spots:
            if hiding_spot is not self.best_hiding_spot:
                egi.cross(hiding_spot, 10)

        # # draw obstacle avoidance box
        # egi.red_pen()
        # pts = [
        #         Point2D(self.pos.x + self.detection_box_length(),  self.pos.y + self.radius),
        #         Point2D(self.pos.x + self.detection_box_length(),  self.pos.y - self.radius),
        #         Point2D(self.pos.x - self.radius,  self.pos.y - self.radius),
        #         Point2D(self.pos.x - self.radius,  self.pos.y + self.radius)
        #     ]
        # #pts = self.world.transform_points(pts, self.pos, self.heading, self.side, self.scale_vector)
        # egi.closed_shape(pts)

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

    # The motion behaviours of Agent ------------------------------------------------------------------

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
            speed = min(speed, self.max_speed)
            # from here proceed just like Seek except we don't need to
            # normalize the to_target vector because we have already gone to the
            # trouble of calculating its length for dist.
            desired_vel = to_target * (speed / dist)
            return (desired_vel - self.vel)
        return Vector2D(0, 0)

    #def avoid_obstacles(self, obstacles):

        # attempted avoid obstacles code from the lecture notes. hasn't worked very well so far

        # box_length = self.detection_box_length()
        # # note (tag) the objects in range
        # #tag_list = self.tag_objects_in_view_range(obstacles, box_length) #####?
        # tag_list = []
        # for obstacle in obstacles:
        #     # if obstacle.pos within self.radius of self.y and obstacle.pos within box_length of self.x???
        #     if obstacle.pos.y <= self.pos.y + self.radius and obstacle.pos.x <= self.pos.x + box_length:
        #         tag_list.append(obstacle)

        # closest_dist = float(99999999999999)
        # closest_obst = None
        # closest_pos = None

        # for obstacle in tag_list:
        #     local_pos = obstacle.pos#self.world.transform_point(obstacle.pos, self.pos, self.heading, self.side) #####?
        #     if local_pos.x >= 0:
        #         expanded_radius = obstacle.b_radius + self.b_radius
        #         if abs(local_pos.y) < expanded_radius:
        #             # line / circle intersection test, x = cx +/- sqrt(r**2 - cy**2) for y = 0
        #             cx = local_pos.x
        #             cy = local_pos.y
        #             # only calc the sqrt part once (avoid repetition)
        #             sqrt_part = sqrt(expanded_radius**2 - cy**2)
        #             ip = cx - sqrt_part
        #             if ip < closest_dist:
        #                 closest_dist = ip
        #                 closest_obst = obstacle
        #                 closest_pos = obstacle.pos

        # # calculate steering force (if required)
        # force = Vector2D()
        # if closest_obst:
        #     # the closer, the stronger the force needed
        #     multi = 1.0 + (box_length - closest_pos.x) / box_length
        #     # lateral force as needed
        #     force.y = (closest_obst.b_radius - closest_pos.y) * multi
        #     # breaking force proportionate to closest obstacle
        #     breaking_weight = 0.2
        #     force.x = (closest_obst.b_radius - closest_pos.x) * breaking_weight

        # # convert force back to world space
        # return self.world.transform_point(force, self.pos, self.heading, self.side) #####?

    def flee(self, hunter_pos, delta):
        ''' move away from hunter position '''
        if self.distance(hunter_pos) > self.avoidance_range:
            return self.wander(delta)
        else:
            desired_vel = (self.pos - obstacle_pos).normalise() * self.max_speed
            return (desired_vel - self.vel)

    def follow_path(self):
        if self.path.current_pt() is self.path.end_pt():
            return self.arrive(self.path.current_pt(), "slow")
        else:
            dist = self.distance(self.path.current_pt())
            if self.distance(self.path.current_pt()) < self.waypoint_threshold:
                self.path.inc_current_pt()
            
            if self.distance(self.path.current_pt()) < self.waypoint_threshold * 3:
                return self.arrive(self.path.current_pt(), "slow")
            else:
                return self.seek(self.path.current_pt())

    def hide(self, hunter, obstacles, delta):
        dist_to_closest = 999999999999999999999999.0
        best_hiding_spot = None
        self.hiding_spots = []
        self.best_hiding_spot = None

        # check for possible hiding spots
        for obstacle in obstacles:
            hiding_spot = self.get_hiding_position(hunter, obstacle)
            self.hiding_spots.append(hiding_spot)
            hiding_dist = self.distance(obstacle.pos)
            if hiding_dist < dist_to_closest:
                dist_to_closest = hiding_dist
                best_hiding_spot = hiding_spot

        if best_hiding_spot:
            self.best_hiding_spot = best_hiding_spot
            return self.arrive(best_hiding_spot, 'fast')

        # default: run away
        return self.flee(hunter.pos, delta)

    def pursuit(self, evader):
    	''' this behaviour predicts where an agent will be in time T and seeks
    		towards that point to intercept it. '''
    	to_evader = evader.pos - self.pos
    	relative_heading = self.heading.dot(evader.heading)

    	if (to_evader.dot(self.heading) > 0) and (relative_heading < 0.95):
    		return self.seek(evader.pos)

    	future_time = to_evader.length()/(self.max_speed + evader.speed())
    	#future_time += (1 - self.heading.dot(evader.vel))
    	future_pos = evader.pos + evader.vel * future_time
    	return self.seek(future_pos)

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

    def wander(self, delta):
        if self.mode == 'flee':
            self.wander_while_fleeing = True
        ''' Random wandering using a projected jitter circle. '''
        wt = self.wander_target
        # this behaviour is dependant on the update rate, so this line must
        # be included when using time independent framerates.
        jitter_tts = self.wander_jitter * delta # this time slice
        # first, add a small random vector to the target's position
        wt += Vector2D(uniform(-1, 1) * jitter_tts, uniform(-1, 1) * jitter_tts)
        # re-project this new fector back onto a unit circle
        wt.normalise()
        # increase the length of the vector to the same as the radius
        # of the wander circle
        wt *= self.wander_radius
        # move the target into a position WanderDist in front of the agent
        target = wt + Vector2D(self.wander_dist, 0)
        # project the target into world space
        wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)
        # and steer towards it
        force = self.seek(wld_target)
        return force 

    # Additional assistive methods used by Agent ------------------------------------------------------

    def apply_friction(self):
        future_pos = self.pos + self.vel * 0.1
        accel_to_future_pos = self.seek(future_pos)
        friction = accel_to_future_pos * -self.friction
        return friction

    # def detection_box_length(self):
    #     # calculate a detection box length proportionate to the current speed
    #     min_box_length = 10.0
    #     return min_box_length + (self.speed() / self.max_speed) * min_box_length

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

    def get_hiding_position(self, hunter, obstacle):
        # set the distance between the obstacle and the hiding point
        dist_from_boundary = 30.0 # system setting
        dist_away = obstacle.radius + dist_from_boundary

        # get the normal vector from the hunter to the hiding point
        to_obstacle = obstacle.pos - hunter.pos
        to_obstacle.normalise()

        # scale size past the obstacle to the hiding location
        return (to_obstacle * dist_away) + obstacle.pos

    def randomise_path(self):
        num_pts = 4
        cx = self.world.cx
        cy = self.world.cy
        margin = min(cx, cy) * (1/6)    #use this for padding in the next line
        self.path.create_random_path(num_pts, margin, margin, cx - margin, cy - margin)

    def speed(self):
        return self.vel.length()
