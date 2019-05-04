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

    def __init__(self, world=None, scale=30.0, mass=1.0, mode='prey', radius=30.0):
        # keep a reference to the world object
        self.world = world
        self.mode = mode

        # what am I and where am I going?
        dir = radians(random()*360)
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.force = Vector2D()  # current steering force
        self.accel = Vector2D() # current acceleration due to force
        self.radius = radius

        # force- and speed-limiting variables
        self.max_force = 500.0
        self.avoidance_range = 100.0

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(self.scale_scalar, self.scale_scalar)  # easy scaling of agent size

        # wander variables
        self.wander_dist = 2.0 * self.scale_scalar
        self.wander_radius = 1.0 * self.scale_scalar
        self.wander_jitter = 10.0 * self.scale_scalar
        self.wander_target = Vector2D(1.0, 0)
        self.b_radius = self.scale_scalar
        self.wandering = False

        # avoidance variables
        self.avoid_radius = 1.5 * self.radius
        self.sensor_pos = Vector2D()
        self.obst_detected = False
        self.sensor_obst_detected = False
        self.fov_markers = []
        self.neighbours = []

        # group behaviour force multipliers
        self.alignment_multiplier = 1.0
        self.cohesion_multiplier = 1.0
        self.fleeing_multiplier = 1.0
        self.obstacle_avoidance_multiplier = 2.0
        self.separation_multiplier = 1.0
        self.wander_multiplier = 1.0
        self.weight_running_sum = False

        # where am i?
        self.pos = self.get_random_valid_position(world.cx, world.cy, self.world.wall_margin + self.radius, self.world.obstacles, self.world.agents)

        if mode == 'predator':
            self.colour = "RED"
        elif mode == 'prey':
            self.colour = "BLUE"
        else:
            self.colour = "GREEN"

        # to make max speed easier to display, going to restrict the model type to 'dart' 
        self.mass = 1.0
        # limits?
        self.max_speed = 6 * self.scale_scalar
        # data for drawing this agent
        self.vehicle_shape = [
            Point2D(-1.0,  0.6),
            Point2D( 1.0,  0.0),
            Point2D(-1.0, -0.6)
        ]

    # The central logic of the Agent class ------------------------------------------------
    def add_force(self, length, multiplier):
        if multiplier > 0 and (not self.weight_running_sum or length < self.max_force):
            return True

        return False

    def calculate(self, delta):
        # reset the steering force
        mode = self.mode      

        if mode == 'predator':
            force = self.calculate_predator(delta)
        elif mode == 'prey':
            force = self.calculate_prey(delta)
        else:
            force = Vector2D()

        return force

    def calculate_predator(self, delta):
        force = self.avoid_obstacles(self.world.obstacles) * self.obstacle_avoidance_multiplier
        force += self.avoid_walls(self.world.walls) * self.obstacle_avoidance_multiplier
        
        # if force.length() < self.max_force:
        #     force += self.avoid_agents(self.world.agents)        
        
        if force.length() == 0: #self.max_force:
            force = self.wander(delta)

        return force

    def calculate_prey(self, delta):
        force = Vector2D()

        if self.add_force(force.length(), 1):
            force += self.avoid_obstacles(self.world.obstacles) * self.obstacle_avoidance_multiplier
            force += self.avoid_walls(self.world.walls) * self.obstacle_avoidance_multiplier
        
        # if self.add_force(force.length(), 1):
        #     force += self.avoid_agents(self.world.agents)

        if self.add_force(force.length(), self.fleeing_multiplier):
            force += self.flee(self.world.predator.pos, delta) * self.fleeing_multiplier

        if self.add_force(force.length(), self.separation_multiplier):
            force += self.separation() * self.separation_multiplier

        if self.add_force(force.length(), self.cohesion_multiplier):
            force += self.cohesion() * self.cohesion_multiplier

        if self.add_force(force.length(), self.alignment_multiplier):
            force += self.alignment() * self.alignment_multiplier

        if self.add_force(force.length(), self.wander_multiplier):
            force += self.wander(delta) * self.wander_multiplier

        return force

    def update(self, delta):
        ''' update vehicle position and orientation '''
        self.obst_detected = False
        self.sensor_obst_detected = False

        # determine the new force
        self.force = self.calculate(delta)      
        self.force.truncate(self.max_force)

        # determine the new accelteration
        self.accel = self.force * (1 / self.mass)  # not needed if mass = 1.0

        # new velocity
        self.vel = self.vel + self.accel * delta
        self.vel.truncate(self.max_speed)
        
        # update position
        pos = self.pos
        self.pos += self.vel * delta

        # update heading is non-zero velocity (moving)
        if self.vel.lengthSq() > 0.00000001:
            self.heading = self.vel.get_normalised()
            self.side = self.heading.perp()
        
        # treat world as continuous space - wrap new position if needed
        self.world.wrap_around(self.pos)

        # if collided, apply changes taking into account collided objects
        if self.collided(self.pos):
            self.pos = pos

    def render(self, color=None):
        if self.world.show_avoid:
            if self.obst_detected:
                egi.set_pen_color(name='RED')
            else:
                egi.set_pen_color(name='LIGHT_BLUE')
            egi.circle(self.pos, self.avoid_radius)


            if self.sensor_obst_detected:
                egi.set_pen_color(name='RED')
            else:
                egi.set_pen_color(name='LIGHT_BLUE')
            egi.circle(self.sensor_pos, self.avoid_radius)

        if self.world.show_radius:
            if not self.collided(self.pos):
                egi.set_pen_color(name='LIGHT_BLUE')
            else:
                egi.red_pen()
            egi.circle(self.pos, self.radius)

        if self.world.show_neighbourhood and self.mode == 'prey':
            egi.green_pen()
            egi.circle(self.pos, self.world.neighbourhood_radius)

        # draw wander info if wandering
        if self.world.show_wander:
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

        egi.set_pen_color(name=self.colour)
        egi.set_stroke(2)
        pts = self.world.transform_points(self.vehicle_shape, self.pos, self.heading, self.side, self.scale_vector)
        egi.closed_shape(pts)

        # add some handy debug drawing info lines - force and velocity
        if self.world.show_forces:
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

    def alignment(self):
        avg_heading = Vector2D()
        avg_count = 0

        for agent in self.neighbours:
            avg_heading += agent.heading
            avg_count += 1

        if avg_count > 0:
            avg_heading *= (1 / float(avg_count))
            avg_heading -= self.heading

        return avg_heading

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
    
    def avoid(self, obj_pos):
        # velocity-based
        desired_vel = (self.pos - obj_pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

        # force-based
        # max-speed
        # desired_force = (self.pos - obj_pos).normalise() * self.max_speed
        # proportionate force
        # desired_force = (self.pos - obj_pos).normalise() * (self.pos - obj_pos).length()
        # return (desired_force - self.force)
    
    # def avoid_agents(self, agents):
    #     agts = agents.copy()
    #     agts.remove(self)
    #     return self.avoid_obstacles(agts)  

    def avoid_obstacles(self, obstacles):
        sns_pos = Vector2D(min(self.avoid_radius * 2, self.vel.length()), 0) # factor in current speed
        self.sensor_pos = self.world.transform_point(sns_pos, self.pos, self.heading, self.side)

        closest_obst = None
        closest_dist = 9999999999999

        closest_obst_sns = None
        closest_dist_sns = 9999999999999

        result = Vector2D(0, 0)

        for obstacle in obstacles:
            dist_to_self = self.distance(obstacle.pos)
            dist_to_avoid = (obstacle.pos - self.sensor_pos).length()

            if dist_to_self < self.avoid_radius + obstacle.radius and dist_to_self < closest_dist:
                closest_obst = obstacle
                closest_dist = dist_to_self

            if dist_to_avoid < self.avoid_radius + obstacle.radius and dist_to_avoid < closest_dist_sns:
                closest_obst_sns = obstacle
                closest_dist_sns = dist_to_avoid

        if closest_obst is not None:
            self.obst_detected = True
            result += self.avoid(closest_obst.pos)

        if closest_obst_sns is not None:
            self.sensor_obst_detected = True
            result += self.avoid(closest_obst_sns.pos)

        return result

    def avoid_walls(self, walls):
        sns_pos = Vector2D(min(self.avoid_radius * 2, self.vel.length()), 0) # factor in current speed
        self.sensor_pos = self.world.transform_point(sns_pos, self.pos, self.heading, self.side)

        closest_wall = None
        closest_dist = 9999999999999

        closest_wall_sns = None
        closest_dist_sns = 9999999999999

        result = Vector2D(0, 0)

        for wall in walls:
            dist_to_self = self.distance(wall.get_pos(self.pos))
            dist_to_avoid = (wall.get_pos(self.sensor_pos) - self.sensor_pos).length()

            if dist_to_self < self.avoid_radius and dist_to_self < closest_dist:
                closest_wall = wall
                closest_dist = dist_to_self

            if dist_to_avoid < self.avoid_radius and dist_to_avoid < closest_dist_sns:
                closest_wall_sns = wall
                closest_dist_sns = dist_to_avoid

        if closest_wall is not None:
            self.obst_detected = True
            result += self.avoid(closest_wall.get_pos(self.pos))

        if closest_wall_sns is not None:
            self.sensor_obst_detected = True
            result += self.avoid(closest_wall_sns.get_pos(self.pos))

        return result

    def cohesion(self):
        centre_mass = Vector2D()
        force = Vector2D()
        avg_count = 0

        for agent in self.neighbours:
            centre_mass += agent.pos
            avg_count += 1

        if avg_count > 0:
            centre_mass *= (1 / float(avg_count))
            force = self.seek(centre_mass)

        return force

    def flee(self, hunter_pos, delta):
        ''' move away from hunter position '''
        if self.distance(hunter_pos) > self.avoidance_range:
            return self.wander(delta)
        else:
            return self.avoid(hunter_pos)

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

    def separation(self):
        force = Vector2D()
        avoid = []

        for agent in self.neighbours:
            # force += self.avoid(agent)

            to_agent = self.pos - agent.pos
            force += to_agent.get_normalised() * (1 / to_agent.length())

        return Vector2D()

    def wander(self, delta):
        self.wandering = True
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

    def collided(self, pos):
        for obstacle in self.world.obstacles:
            if obstacle.distance(pos) < self.radius + obstacle.radius:
                return True

        for agent in self.world.agents:
            if agent is not self and agent.distance(pos) < self.radius + agent.radius:
                return True

        return False

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

    def get_random_valid_position(self, max_x, max_y, margin, obstacles, agents):
        valid = False
        pos = Vector2D()

        while not valid:
            valid = True
            pos = Vector2D(randrange(margin, max_x - margin), randrange(margin, max_y - margin))
            
            for obstacle in obstacles:
                if obstacle.distance(pos) <= self.avoid_radius + obstacle.radius:
                    valid = False

            for agent in agents:
                if agent is not self and agent.distance(pos) <= self.avoid_radius + agent.avoid_radius:
                    valid = False

        return pos
      
    # The main function that returns true if line segment 'p1q1' 
    # and 'p2q2' intersect. 
    # Borrowed from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    def intersect(self, p1, q1, p2, q2): 
        # Given three colinear points p, q, r, the function checks if 
        # point q lies on line segment 'pr' 
        def on_segment(p, q, r): 
            if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)): 
               return True 
          
            return False       
      
        # To find orientation of ordered triplet (p, q, r). 
        # The function returns following values 
        # 0 --> p, q and r are colinear 
        # 1 --> Clockwise 
        # 2 --> Counterclockwise 
        def orientation(p, q, r): 
            val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
          
            if val == 0:
                return 0    # colinear 
            elif val > 0:
                return 1    # clockwise 
            else:
                return 2    # anticlockwise

        # Find the four orientations needed for general and 
        # special cases 
        o1 = orientation(p1, q1, p2) 
        o2 = orientation(p1, q1, q2) 
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)
      
        # General case 
        if (o1 is not o2 and o3 is not o4): 
            return True 
      
        # Special Cases 
        # p1, q1 and p2 are colinear and p2 lies on segment p1q1 
        if o1 == 0 and on_segment(p1, p2, q1):
            return True
      
        # p1, q1 and q2 are colinear and q2 lies on segment p1q1 
        if o2 == 0 and on_segment(p1, q2, q1):
            return True
      
        # p2, q2 and p1 are colinear and p1 lies on segment p2q2 
        if o3 == 0 and on_segment(p2, p1, q2):
            return True
      
        # p2, q2 and q1 are colinear and q1 lies on segment p2q2 
        if o4 == 0 and on_segment(p2, q1, q2):
            return True
      
        return False # Doesn't fall in any of the above cases 

    def speed(self):
        return self.vel.length()

    # def update_multiplier(self, multiplier, change):
    #     # group behaviour force multipliers
    #     if multiplier == 'alignment':
    #         self.alignment_multiplier += change

    #         if self.alignment_multiplier < 0:
    #             self.alignment_multiplier = 0
    #     elif multiplier == 'cohesion':
    #         self.cohesion_multiplier += change  
            
    #         if self.cohesion_multiplier < 0:
    #             self.cohesion_multiplier = 0                
    #     elif multiplier == 'fleeing':
    #         self.fleeing_multiplier += change

    #         if self.fleeing_multiplier < 0:
    #             self.fleeing_multiplier = 0
    #     elif multiplier == 'separation':
    #         self.separation_multiplier += change

    #         if self.separation_multiplier < 0:
    #             self.separation_multiplier = 0
    #     elif multiplier == 'wander':
    #         self.wander_multiplier += change

    #         if self.wander_multiplier < 0:
    #             self.wander_multiplier = 0

