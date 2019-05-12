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
from hiding_spot import HidingSpot
from projectile import Projectile
from datetime import datetime, time, timedelta

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

    def __init__(self, world=None, scale=30.0, mass=1.0, agent_type=None, mode=None, sub_mode=None, weapon=None, speed_limiter=5, radius=30.0, pos=Vector2D()):
        # keep a reference to the world object
        self.world = world
        self.agent_type = agent_type
        self.mode = mode
        self.mode_index = -1 
        self.sub_mode = sub_mode
        self.weapon = weapon
        self.weapon_index = -1 

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(scale, scale)  # easy scaling of agent size

        # what am I and where am I going?
        dir = radians(random()*360)
        self.mass = 1.0
        self.vel = Vector2D(0,0)
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.force = Vector2D(0,0)  # current steering force
        self.accel = Vector2D(0,0) # current acceleration due to force
        self.radius = radius
        self.hiding_spots = []
        self.best_hiding_spot = None

        # force- and speed-limiting variables
        self.max_force = 500.0
        self.avoidance_range = 100.0
        self.speed_limiter = speed_limiter
        self.max_speed = 30 * self.scale_scalar / speed_limiter

        # path variables
        # self.path = Path()
        # self.randomise_path()
        # self.waypoint_threshold = 40.0

        # wander variables
        self.wander_dist = 2.0 * self.scale_scalar
        self.wander_radius = 1.0 * self.scale_scalar
        self.wander_jitter = 10.0 * self.scale_scalar
        self.wander_target = Vector2D(1.0, 0)
        self.b_radius = self.scale_scalar
        self.wandering = False

        # avoidance variables
        self.avoid_radius = 2 * self.radius
        self.sensor_pos = Vector2D()
        self.obst_detected = False
        self.sensor_obst_detected = False
        self.fov_markers = []
        self.see_target = False

        # debug draw info?
        self.show_info = False
        self.show_avoidance = False

        # where am i?
        if pos is None:
        	self.pos = self.get_random_valid_position(world.cx, world.cy, self.world.obstacles, self.world.agents)
        else:
        	self.pos = pos

        # projectile variables
        self.hit_time = None

        self.projectile_pool = []
        i = 0
        while i < 200:
        	self.projectile_pool.append(Projectile(world=self.world, shooter=self))
        	i += 1

        self.current_pt = None
        self.next_pt = None

        self.last_shot = datetime.now()
        self.cooldown = 0
        self.effective_range = 0
        self.hunt_dist = 0

        self.next_weapon()

    # The central logic of the Agent class ------------------------------------------------

    def update(self, delta):
        ''' update vehicle position and orientation '''
        if self.hit_time is not None and (datetime.now() - self.hit_time).total_seconds() > 0.1:
            self.hit_time = None

        self.obst_detected = False
        self.sensor_obst_detected = False

        if self.moving():
            self.move(delta)

        if self.agent_type == 'shooter':
        	if self.world.target.mode == 'Evading' and self.world.obstacles_enabled and not self.see_target:
        	 	return
        	elif (datetime.now() - self.last_shot).total_seconds() > self.cooldown:
        		self.last_shot = datetime.now()
        		self.shoot(self.world.target)

    def moving(self):
    	if self.agent_type == 'shooter' and self.world.target.mode == 'Evading':
    		return True

    	elif self.agent_type == 'target' and self.mode is not 'Stationary':
    		return True
    
    	return False

    def move(self, delta):
    	if self.agent_type == 'target':
	    	sns_pos = Vector2D(min(self.avoid_radius * 2, self.vel.length()), 0) # factor in current speed
	    	self.sensor_pos = self.world.transform_point(sns_pos, self.pos, self.heading, self.side)

    	force = Vector2D()

    	# disabled obstacle avoidance; hinders movement in this scenario
    	# if self.world.obstacles_enabled:
    	# 	force += self.avoid_obstacles(self.world.obstacles)

    	if self.world.walls_enabled:
    		force += self.avoid_walls(self.world.walls) * 10

    	force += self.avoid_agents(self.world.agents)

    	if force.length() == 0:
    		force = self.calculate(delta)  

    	## limit force
    	force.truncate(self.max_force)
    	self.force = force

    	# determine the new accelteration
    	self.accel = force * (1 / self.mass)  # not needed if mass = 1.0

    	# new velocity
    	self.vel += self.accel * delta
    	self.vel.truncate(self.max_speed)
    
    	# update position
    	pos = self.pos
    	accel = self.accel
    	vel = self.vel
    	force = self.force
    	self.pos += self.vel * delta
    
    	# update heading is non-zero velocity (moving)
    	if self.vel.lengthSq() > 0.00000001:
    		self.heading = self.vel.get_normalised()
    		self.side = self.heading.perp()
    
    	# treat world as continuous space - wrap new position if needed
    	self.world.wrap_around(self.pos)

    	if self.collided():
    		self.pos = pos
    		self.accel = accel
    		self.vel = vel
    		self.force = force

    	    # update heading is non-zero velocity (moving)
    		if self.vel.lengthSq() > 0.00000001:
    			self.heading = self.vel.get_normalised()
    			self.side = self.heading.perp()
        
    	    # treat world as continuous space - wrap new position if needed
    		self.world.wrap_around(self.pos)

    def calculate(self, delta):
        # reset the steering force
        agent_type = self.agent_type

        if agent_type == 'target':
            force = self.calculate_target(delta)
        elif agent_type == 'shooter':
            force = self.calculate_shooter(delta)
        else:
            force = Vector2D(0,0)

        return force

    def calculate_target(self, delta):
    	if self.mode == 'Pacing' or self.mode == 'Evading':
    		avoid = self.avoid_projectiles(self.world.projectiles)
    		if avoid.length() > 0:
    			print('dodging projectile')
    			return avoid

    		if self.mode == 'Pacing':
    			return self.pace(delta)
    		elif self.mode == 'Evading':
    			if self.world.obstacles_enabled:
    				return self.hide(self.world.shooter, self.world.hiding_spots, delta)
    			else:
    				return self.flee(self.world.shooter.pos, delta)

    	elif self.vel.length() > 0:
    		return self.vel * -1

    		return Vector2D(0,0)

    def calculate_shooter(self, delta):
    	self.update_fov(self.world.obstacles, self.world.agents)

    	if self.world.target.mode == 'Evading':
    		if self.world.obstacles_enabled:
    			return self.hunt(self.world.target, delta)
    		else:
    			return self.seek(self.world.target.pos)

    	return Vector2D(0,0)

    def render(self, color=None):
        if self.show_avoidance or self.agent_type == 'target':
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

        # # draw the path if it exists and the mode is follow
        # if self.mode == 'follow_path':
        #     self.path.render()
        
        # # draw the ship according to its status if pursuing
        # elif self.world.agent_mode == 'pursuit':
        #   if self.mode == 'pursuit':
        #       egi.set_pen_color(name='RED')
        #   else:
        #       egi.set_pen_color(name='WHITE')
        
        #el

        # # draw wander info if wandering
        # if self.mode == 'wander' or self.wandering:
        #     # calculate the centre of the wander circle in front of the agent
        #     wnd_pos = Vector2D(self.wander_dist, 0)
        #     wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)

        #     # draw the wander circle
        #     egi.green_pen()
        #     egi.circle(wld_pos, self.wander_radius)

        #     # draw the wander target (little circle on the big circle)
        #     egi.red_pen()
        #     wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
        #     wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
        #     egi.circle(wld_pos, 3)

        # draw the ship 
        egi.set_stroke(2)

        if self.hit_time is not None:
            egi.red_pen()
        else:
            egi.orange_pen()

        if self.agent_type == 'target':
        	egi.circle(self.pos, self.radius)
        	egi.circle(self.pos, self.radius * 2 / 3)
        	egi.circle(self.pos, self.radius / 3)
        	egi.cross(self.pos, self.radius)

        	if self.mode == 'Pacing':
        		egi.red_pen()
        		egi.cross(self.current_pt, 10)
        		egi.circle(self.current_pt, 10)
        		egi.cross(self.next_pt, 10)
        else:
            egi.circle(self.pos, self.radius)

            if self.world.target.mode == 'Evading':
                egi.set_pen_color(name='AQUA')
                egi.circle(self.pos, self.effective_range)

                if self.world.obstacles_enabled and len(self.fov_markers) >= 3:
                    # draw the field of view
                    if self.see_target:
                        egi.red_pen()
                    else:
                        egi.set_pen_color(name='WHITE')
                    egi.circle(self.pos, self.hunt_dist)
                    egi.line(pos1=self.pos, pos2=self.fov_markers[0])
                    egi.line(pos1=self.pos, pos2=self.fov_markers[2])
                    egi.line(pos1=self.fov_markers[0], pos2=self.fov_markers[2])

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
    
    def avoid(self, obj_pos):
        desired_vel = (self.pos - obj_pos).normalise() * self.max_speed
        return (desired_vel - self.vel)
    
    def avoid_agents(self, agents):
        agts = agents.copy()
        agts.remove(self)
        return self.avoid_obstacles(agts)  

    def avoid_obstacles(self, obstacles):
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

    def avoid_projectiles(self, projectiles):
        def avoid_projectile(self, projectile):
            to_projectile = projectile.pos - self.pos
            desired_vel = to_projectile.perp() * self.max_speed
            print('desired vel is ' + str(desired_vel))
            return (desired_vel - self.vel)

        print('checking for projectiles')

        closest_proj = None
        closest_dist = 9999999999999

        closest_proj_sns = None
        closest_dist_sns = 9999999999999

        result = Vector2D(0,0)

        for projectile in projectiles:
            dist_to_self = self.distance(projectile.pos)
            dist_to_avoid = (projectile.pos - self.sensor_pos).length()

            if dist_to_self < self.avoid_radius + projectile.radius and dist_to_self < closest_dist:
                closest_proj = projectile
                closest_dist = dist_to_self

            if dist_to_avoid < self.avoid_radius + projectile.radius and dist_to_avoid < closest_dist_sns:
                closest_proj_sns = projectile
                closest_dist_sns = dist_to_avoid

        if closest_proj is not None:
            self.obst_detected = True
            result += avoid_projectile(self, closest_proj)
        
        if closest_proj_sns is not None:
            self.sensor_obst_detected = True
            result += avoid_projectile(self, closest_proj_sns)

        return result

    def avoid_walls(self, walls):
        closest_wall = None
        closest_dist = 9999999999999

        closest_wall_sns = None
        closest_dist_sns = 9999999999999

        result = Vector2D(0, 0)

        for wall in walls:
            wall_pos = wall.get_pos(self.pos)
            dist_to_self = self.distance(wall_pos)
            dist_to_avoid = (wall_pos - self.sensor_pos).length()

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

    def flee(self, hunter_pos, delta):
        ''' move away from hunter position '''
        if self.distance(hunter_pos) > self.avoidance_range:
            return self.wander(delta)
        else:
            return self.avoid(hunter_pos)

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

    def hide(self, hunter, hiding_spots, delta):
        self.best_hiding_spot = None        
        
        # if only one hiding spot, just pick that spot 
        if len(hiding_spots) == 1:            
            self.best_hiding_spot = hiding_spots[0]
        elif len(hiding_spots) > 1:  
            # go to best hiding spot
            self.best_hiding_spot = self.get_best_hiding_spot(hiding_spots, hunter)
        
        if self.best_hiding_spot is None:
            # default: panic and run away from a random hunter
            return self.flee(hunter.pos, delta)
        else:
            self.best_hiding_spot.best = True    
            return self.arrive(self.best_hiding_spot.pos, 'fast')          

    def hunt(self, evader, delta):
        prioritise_visible = False
        prioritise_close = False

        self.see_target = True

        hunting = []
        visible = []
        close = []
        dist_multiplier = 1.25

        if self.hunt_dist < (self.avoid_radius + evader.avoid_radius) * dist_multiplier - evader.avoid_radius:
        	self.hunt_dist = (self.avoid_radius + evader.avoid_radius) * dist_multiplier - evader.avoid_radius

        if self.distance(evader.pos) < self.hunt_dist + evader.avoid_radius:
            close.append(evader)

        for marker in self.fov_markers:
            if evader.distance(marker) < evader.avoid_radius: #* dist_multiplier
                visible.append(evader)

        hunting = list(set(visible) and set(close))

        visible.sort(key=lambda x: x.distance(self.pos))

        if len(hunting) > 0:
            hunting.sort(key=lambda x: x.distance(self.pos))  
            return self.pursuit(hunting[0])
        elif prioritise_close and len(close) > 0:
            if len(close) > 1:
                close.sort(key=lambda x: x.distance(self.pos))
            return self.pursuit(close[0])
        elif prioritise_visible and len(visible) > 0:
            if len(visible) > 1:
                visible.sort(key=lambda x: x.distance(self.pos))
            return self.pursuit(visible[0])
        else:
            hunting = close + visible
            if len(hunting) == 0:
                self.see_target = False
                return self.wander(delta)
            else:
                if len(hunting) > 1:
                    hunting.sort(key=lambda x: x.distance(self.pos))                  
                return self.pursuit(hunting[0])

    def pace(self, delta):
    	threshold = 10

    	if self.current_pt is not None and self.next_pt is not None:
    		if self.distance(self.current_pt) < threshold:
    			temp = self.current_pt
    			self.current_pt = self.next_pt
    			self.next_pt = temp

    		return self.arrive(self.current_pt, 'fast')
    	else:
    		return Vector2D()

    def pursuit(self, evader):
    	''' this behaviour predicts where an agent will be in time T and seeks
    		towards that point to intercept it. '''
    	to_evader = evader.pos - self.pos
    	relative_heading = self.heading.dot(evader.heading)

    	if (to_evader.dot(self.heading) > 0) and (relative_heading < 0.95):
    		return self.seek(evader.pos)

    	future_time = to_evader.length()/(self.max_speed + evader.speed())
    	#future_time += (1 - self.heading.dot(evader.vel)) * - self.side.length()
    	future_pos = evader.pos + evader.vel * future_time
    	return self.seek(future_pos)

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

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

    # Marksmanship Methods ------------------------------------------------------------------------

    def get_future_pos_with_accel(self, start_vel, accel, time):
    	displacement = (start_vel * time) + (0.5 * accel * time * time) #d = ut + 0.5at^2
    	return displacement 

    def shoot(self, target):
    	slow_speed = 250
    	fast_speed = 1000
    	mod = 50

    	# check if any projectiles are still pooled
    	if len(self.projectile_pool) > 0:
    		# get speed according to weapon type
    		if self.weapon == 'Rocket' or self.weapon == 'Hand Grenade':
    			p_speed = slow_speed
    		else:
    			p_speed = fast_speed

    		print("stationary: " + str(target.pos))

    		if target.mode == 'Stationary':
    			target_pos = target.pos.copy()
    		else:
    			''' this behaviour predicts where an agent will be in time T and seeks
    			towards that point to intercept it. '''
    			to_target = target.pos - self.pos
    			future_target_pos = target.pos.copy()
    			dist = to_target.length()
    			loop = True
    			loop_count = 0

    			# loops its predictive logic several times to get progressively better predictions. If it would
    			# get to an iteration where the distance between the projectile and target would be worse than
    			# on the last iteration, it uses the last iteration
    			while loop:
    				loop_count += 1
    				future_time = (future_target_pos - self.pos).length()/(p_speed)		# first loop: current target.pos
    				future_target_pos = target.pos + self.get_future_pos_with_accel(target.vel, target.accel, future_time)

    				# if it's not evading, it's not gonna move outside the bounds of the window
    				if target.mode == 'Evading':
    					self.world.wrap_around(future_target_pos)

    				vel_to_future_pos = (future_target_pos - self.pos).normalise() * p_speed
    				future_self_pos = self.pos + vel_to_future_pos * future_time
    				new_dist = (future_target_pos - future_self_pos).length()

    				# keep iterating?
    				if new_dist < dist: 
    					dist = new_dist

    					# is it's predicted pos close enough?
    					if dist < target.radius * 0.1:
    						print('dist between predicted positions less than 0.1 times the targets radius')
    						loop = False

    				else:
    					loop = False

    			target_pos = future_target_pos
    			print("predictive: " + str(target_pos))
    			print("loop count: " + str(loop_count))

    		if self.world.target.mode == 'Evading' and self.distance(target_pos) > self.effective_range + target.radius:
    			return

    		if self.weapon == 'Shotgun':
    			# shot gun scatters multiple projectiles
    			loop = 5
    		else:
    			# everything else fires one projectile at a time
    			loop = 1

    		while loop > 0:
    			# affect accuracy if using an inaccurate weapon
    			if self.weapon == 'Hand Gun' or self.weapon == 'Hand Grenade' or self.weapon == 'Shotgun':
    				target_pos.x = int(randrange(int(target_pos.x - mod), int(target_pos.x + mod)))
    				target_pos.y = int(randrange(int(target_pos.y - mod), int(target_pos.y + mod)))

    			# calculate velocity
    			p_heading = (target_pos - self.pos).normalise()
    			p_vel = p_heading * p_speed

    			# set up and shoot projectile
    			p = self.projectile_pool[0]
    			self.projectile_pool.remove(p)
    			p.vel = p_vel
    			p.pos = self.pos.copy()
    			p.p_type = self.weapon

    			# where's the grenade gonna land?
    			if self.weapon == 'Hand Grenade':
    				p.target = target_pos

    			# add new projectile
    			self.world.projectiles.append(p)

    			loop -= 1

    # Additional assistive methods used by Agent --------------------------------------------------

    def collided(self):
        if self.world.obstacles_enabled:
        	for obstacle in self.world.obstacles:
	            if self.distance(obstacle.pos) < self.radius + obstacle.radius:
	                return True

        if self.world.walls_enabled:
	        for wall in self.world.walls:
	            if wall.distance(self.pos) < self.radius:
	                return True

        for agent in self.world.agents:
            if agent is not self and self.distance(agent.pos) < self.radius + agent.radius:
                return True

        return False

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

	    # def eat(self, evaders):
	    #     to_eat = []

	    #     for evader in evaders:
	    #         if self.distance(evader.pos) < self.avoid_radius + evader.avoid_radius:
	    #             for marker in self.fov_markers:
	    #                 if evader.distance(marker) < evader.avoid_radius:
	    #                     to_eat.append(evader)

	    #     for evader in to_eat:
	    #         self.world.destroy(evader)

	    #     del to_eat

    def get_best_hiding_spot(self, spots, hunter):
        # sort and / or rank the hiding spots
        prioritise_close = False
        prioritise_far_from_hunter = False

        for spot in spots:
            spot.rank = 0

        if prioritise_close:
            # get closest
            for spot in spots:
                spot.dist_to_evader = self.distance(spot.pos)                
            spots.sort(key=lambda x: x.dist_to_evader)
        elif prioritise_far_from_hunter:
            # get furthest from hunter
            spots.sort(key=lambda x: x.avg_dist_to_hunter, reverse=True)
        else:
            # sort and increment spots' rank by closeness, lowest (index = 0) to highest (index = len - 1)
            for spot in spots:
                spot.dist_to_evader = self.distance(spot.pos)
            spots.sort(key=lambda x: x.dist_to_evader)
            for i in range(len(spots) - 1):
                spots[i].rank += i

            # sort and increment spots' rank by distance from hunter, highest (index = 0) to lowest (index = len - 1)
            spots.sort(key=lambda x: x.avg_dist_to_hunter, reverse=True)
            for i in range(len(spots) - 1):
                spots[i].rank += i
            
            # account for the number of collisions that have been experienced going to each spot
            # got agent stuck in a loop where it kept changing the hiding spot it was going to every frame or two
            # if False:    
            #     for spot in spots:
            #         spot.rank += spot.collisions

            # increment spots' rank if moving to them would require crossing the hunter's line of sight
            for spot in spots:
                if self.intersect(self.pos, spot.pos, hunter.pos, hunter.fov_markers[1]):
                    spot.rank += 999999999

            # sort spots by rank, lowest (index = 0) to highest (index = len - 1) 
            spots.sort(key=lambda x: x.rank)

        for i in range(0, len(spots) - 1):
            if spots[i].valid:
                return spots[i]

        return None

    def get_random_valid_position(self, max_x, max_y, obstacles, agents):
        valid = False
        pos = Vector2D()

        while not valid:
            valid = True
            pos = Vector2D(randrange(max_x), randrange(max_y))
            
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

    def next_movement_type(self):
    	max_index = 2
    	self.mode_index += 1

    	if self.mode_index > max_index:
    		self.mode_index = 0
    
    	if self.mode_index == 0:
    		self.mode = 'Stationary'
    	elif self.mode_index == 1:
    		self.mode = 'Pacing'
    	elif self.mode_index == 2:
    		self.mode = 'Evading'

    def next_weapon(self):
    	max_index = 4
    	self.weapon_index += 1

    	if self.weapon_index > max_index:
    		self.weapon_index = 0

    	# rate of fire numbers borrowed from weapons in Halo CE where possible and modified for
    	# how fast I'd probably shoot with them
    	if self.weapon_index == 0:
    		self.weapon = 'Rifle'
    		self.cooldown = 1.5 # max rpm of 0.5 sec / round 
    		self.effective_range = 10 * 2300 # effective range 2300 m
    	elif self.weapon_index == 1:
    		self.weapon = 'Rocket'
    		self.cooldown = 1.5 # max rpm of 0.6 sec / round 
    		self.effective_range = 5 * 160 # estimated effective range 160 m
    	elif self.weapon_index == 2:
    		self.weapon = 'Hand Gun'
    		self.cooldown = 0.286 # max rpm
    		self.effective_range = 5 * 122.7 # effective range 122.7 m
    	elif self.weapon_index == 3:
    		self.weapon = 'Hand Grenade'
    		self.cooldown = 2     # estimated max rpm of 2 sec / round 
    		self.effective_range = 5 * 75 # estimated effective range 75 m
    	elif self.weapon_index == 4:
    		self.weapon = 'Shotgun'
    		self.cooldown = 1   # max rpm of 1 sec / round 
    		self.effective_range = 30 * 5 # estimated effective range 5 m	

    def randomise_path(self):
        num_pts = 4
        cx = self.world.cx
        cy = self.world.cy
        margin = min(cx, cy) * (1/6)    #use this for padding in the next line
        self.path.create_random_path(num_pts, margin, margin, cx - margin, cy - margin)

    def speed(self):
        return self.vel.length()

    def update_fov(self, obstacles, agents):
        crossed_obj = False
        max_fov_length = self.avoid_radius * 20
        fov_length = self.radius
        fovm = Vector2D()
        fov_marker = Vector2D()
        markers = []

        if self.world.obstacles_enabled:
        	objects = obstacles + agents
        else:
        	objects = agents.copy()

        if self in objects:
            objects.remove(self)

        while not crossed_obj and fov_length < max_fov_length: 
            fovm = Vector2D(fov_length, 0)
            fov_marker = self.world.transform_point(fovm, self.pos, self.heading, self.side) 
            
            for obj in objects:
                if obj.distance(fov_marker) < obj.radius:
                    crossed_obj = True
                    overshoot = obj.radius - obj.distance(fov_marker)

            if not crossed_obj:
                fov_length += self.radius
            else:
                fov_length -= overshoot

        fov_length = min(fov_length, max_fov_length)
        fovm = Vector2D(fov_length, 0)
        offset = 5 * self.scale_scalar
        m_left = Vector2D(fov_length, -offset)
        m_right = Vector2D(fov_length, offset)
        markers.append(self.world.transform_point(m_left, self.pos, self.heading, self.side))
        markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
        markers.append(self.world.transform_point(m_right, self.pos, self.heading, self.side))
        self.fov_markers = markers

