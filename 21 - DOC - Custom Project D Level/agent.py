'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians, sqrt
from random import random, randrange, uniform
from projectile import Projectile
from datetime import datetime, time, timedelta
from searches import SEARCHES
from pyglet import clock

search_modes = list(SEARCHES.keys())

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

    def __init__(self, world=None, scale=30.0, agent_type="fugitive", movement_mode=None, combat_mode=None, weapons=[], radius=6.0, box=None, path=None, name="Agent", respawnable=False):
        # keep a reference to the world object
        self.world = world
        self.name = name
        self.agent_type = agent_type
        self.movement_mode = movement_mode
        self.combat_mode = combat_mode
        self.weapons = weapons
        self.ready = False

        self.health = 1000
        self.start_health = 1000
        self.respawnable = respawnable

        # if self.agent_type == 'soldier':
        #     self.health = self.health ** 3

        # scaling variables
        self.scale_scalar = scale
        self.scale_vector = Vector2D(scale, scale)  # easy scaling of agent size

        # what am I and where am I going?
        dir = radians(random()*360)
        self.mass = 1.0
        self.vel = Vector2D(0,0)
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.radius = radius
        self.radius_standard = radius

        self.avoidance_range = 200.0
        self.max_speed_standard = 80
        self.max_speed = 80


        # path variables
        self.path = path
        self.waypoint_threshold = 40.0

        # avoidance variables
        self.avoid_radius = 2 * self.radius
        self.avoid_radius_standard = self.avoid_radius
        # self.see_target = False

        # debug draw info?
        self.show_info = False
        self.show_avoidance = False

        # where am i?
        if box == None:
            self.pos = None
            self.box = None
            self.position_in_random_box()
        else:
            self.box = self.world.boxes[box]
            self.pos = self.box.get_vc("agent init")

        # projectile variables
        self.hit_time = None
        self.last_shot = datetime.now()
        self.hunt_dist = 0

        self.fear = 0
        self.last_fear_ping = None

        self.world.change_weapons(self)

        self.last_aware_time = None
        self.last_node_box = None
        self.current_node_box = None
        self.current_node_pos = None

        self.awareness_radius_standard = 100
        self.awareness_radius = 100
        self.awareness_pos = None

        self.target = None
        self.target_enemy = None
        self.following_enemy = False
        self.target_ally = None

        self.last_new_node_time = None

    # The central logic of the Agent class ------------------------------------------------

    def update(self, delta):
        # check if been stuck on the same node too long while not following an enemy
        if self.last_new_node_time is not None and self.movement_mode is not "Stationary" and not self.following_enemy and (datetime.now() - self.last_new_node_time).total_seconds() > 3:
            self.last_new_node_time = datetime.now()
            self.get_new_path()
            print("overriding timed out path with new path")

        # check if should still display that agent was hit by a weapon
        if self.hit_time is not None and (datetime.now() - self.hit_time).total_seconds() > 0.1:
            self.hit_time = None

        # check which update method to call
        if self.agent_type == "soldier":
            self.update_soldier(delta)
        elif self.agent_type == "fugitive":
            self.update_fugitive(delta)

    def update_soldier(self, delta):
        # check if died
        if self.health <= 0:
            print(self.name + ": x_x")
            self.world.destroy_soldier(self)
            return

        self.awareness_pos = Vector2D(self.awareness_radius * 0.625, 0)
        self.awareness_pos = self.world.transform_point(self.awareness_pos, self.pos, self.heading, self.side)
        
        # select movement mode
        if self == self.world.soldiers[len(self.world.soldiers) - 1] and self.squad_overwhelmed():
            if self.movement_mode is not "Getting Reinforcements":
                self.movement_mode = "Getting Reinforcements"
                self.max_speed = self.max_speed_standard * 1.5
                self.get_new_path()
        elif self.see_target():
            self.movement_mode = "Attack"
            self.target_ally = None

            if self.target_enemy == None or self.target_enemy.distance(self.pos) > self.awareness_radius + self.target_enemy.radius:
                self.target_enemy = None

                for agent in self.world.agents:
                    if agent.agent_type is not self.agent_type and (self.target_enemy == None or self.distance(self.target_enemy.pos) > self.distance(agent.pos)) and self.target_and_path_in_range(agent):
                        self.target_enemy = agent

                # print(self.name + " attacking " + self.target_enemy.name)

            if self.target_enemy is not None and self.target_enemy.box is not self.target:
                self.get_new_path()
        elif self.ally_attacking():
            self.movement_mode = "Assist"

            if self.target_ally == None:
                for agent in self.world.agents:
                    if agent.agent_type is not self.agent_type and (self.target_ally == None or self.distance(self.target_ally.pos) > self.distance(agent.pos)):
                        self.target_ally = agent

            if self.target_ally is not None and self.target_ally.box is not self.target:
                self.get_new_path()
        elif self.movement_mode == "Attack" or self.movement_mode == "Assist":
            self.movement_mode = "Scout"
            self.get_new_path()
        elif self.movement_mode is not "Scout" and self.movement_mode is not "Stationary" and self.movement_mode is not "Getting Reinforcements":
            self.movement_mode = "Patrol"

        if self.choose_weapon():
            # set combat mode and handle reloading
            if self.weapons[0].rounds_left_in_magazine == 0 and (datetime.now() - self.last_shot).total_seconds() <= self.weapons[0].reload_time:
                self.combat_mode = 'Reloading'
            else:
                if self.weapons[0].rounds_left_in_magazine == 0:
                    self.weapons[0].rounds_left_in_magazine += self.weapons[0].magazine_size
                    self.weapons[0].magazines_left -= 1

                if (datetime.now() - self.last_shot).total_seconds() <= self.weapons[0].cooldown:
                    self.combat_mode = 'Weapon is Loading Next Round'
                elif self.movement_mode == 'Attack':
                    self.combat_mode = 'Aiming'
                elif self.movement_mode == 'Patrol' or self.movement_mode == "Assist":
                    self.combat_mode = 'Ready'

            # aim and shoot
            if (self.movement_mode == 'Attack' or self.movement_mode == "Getting Reinforcements") and self.combat_mode == 'Aiming':
                if len(self.weapons[0].projectile_pool) == 0:
                    self.combat_mode = 'No Projectiles Pooled'
                elif self.target_enemy is not None:
                    target = self.aim_shot(self.target_enemy)

                    if target is not None:
                        self.combat_mode = 'Shooting'
                        self.last_shot = datetime.now()
                        self.weapons[0].rounds_left_in_magazine -= 1
                        self.shoot(target)
        else:
            self.world.change_weapons(self)
        
        if self.movement_mode == "Attack":
            if self.target_enemy is not None and self.target == self.box:
                self.follow_target_enemy(delta)
            else:
                self.follow_graph_path(delta)
        elif self.movement_mode == "Assist" or self.movement_mode == "Scout":
            self.follow_graph_path(delta)
        elif self.movement_mode == "Patrol":
            if (self is not self.world.soldiers[0] and self.target is not self.world.soldiers[0].target) or (self == self.world.soldiers[0] and self.target not in self.world.waypoints[self.world.current_waypoint].nodes):
                self.get_new_path()
            
            self.follow_graph_path(delta)
        elif self.movement_mode == "Getting Reinforcements":
            if self.target is not self.world.bases[0]:
                self.get_new_path()

            self.follow_graph_path(delta)

        self.update_heading()
        self.box = self.world.get_box_by_pos(int(self.pos.x), int(self.pos.y))

    def update_fugitive(self, delta):
        # check if died
        if self.health <= 0:
            if self.respawnable:
                self.position_in_random_box()
                self.path = None
                self.health = self.start_health
                self.hit_time = None
                self.movement_mode = "Stationary"
                self.fear = 0
                self.last_fear_ping = None
            else:
                print(self.name + ": x_x")
                self.world.destroy_fugitive(self)

        self.awareness_pos = Vector2D(self.awareness_radius * 0.625, 0)
        self.awareness_pos = self.world.transform_point(self.awareness_pos, self.pos, self.heading, self.side)


        if not self.scared():
            if self.see_target():
                self.movement_mode = "Attack"

                if self.target_enemy == None or self.target_enemy.distance(self.pos) > self.awareness_radius + self.target_enemy.radius:
                    self.target_enemy = None

                    for agent in self.world.agents:
                        if agent.agent_type is not self.agent_type and (self.target_enemy == None or self.distance(self.target_enemy.pos) > self.distance(agent.pos)) and self.target_and_path_in_range(agent):
                            self.target_enemy = agent

                    # print(self.name + " attacking " + self.target_enemy.name)

                if self.target_enemy is not None and self.target_enemy.box is not self.target:
                    self.get_new_path()
            elif self.movement_mode is not "Stationary":
                # print("Fugitive lost soldier. Sitting still.")
                self.sit_still()

            if self.choose_weapon():
                if self.weapons[0].rounds_left_in_magazine == 0 and (datetime.now() - self.last_shot).total_seconds() <= self.weapons[0].reload_time:
                    self.combat_mode = 'Reloading'
                else:
                    if self.weapons[0].rounds_left_in_magazine == 0:
                        self.weapons[0].rounds_left_in_magazine += self.weapons[0].magazine_size
                        self.weapons[0].magazines_left -= 1

                    if (datetime.now() - self.last_shot).total_seconds() <= self.weapons[0].cooldown:
                        self.combat_mode = 'Weapon is Loading Next Round'
                    elif self.movement_mode == 'Attack' or "Flee":
                        self.combat_mode = 'Aiming'
                    elif self.movement_mode == "Stationary":
                        self.combat_mode = 'Ready'

                if (self.movement_mode == 'Attack' or self.movement_mode == "Flee") and self.combat_mode == 'Aiming':
                    if len(self.weapons[0].projectile_pool) == 0:
                        self.combat_mode = 'No Projectiles Pooled'
                    elif self.target_enemy is not None:
                        target = self.aim_shot(self.target_enemy)

                        if target is not None:
                            self.combat_mode = 'Shooting'
                            self.last_shot = datetime.now()
                            self.weapons[0].rounds_left_in_magazine -= 1
                            self.shoot(target)
            else:
                self.world.change_weapons(self)
            
        elif self.max_speed == self.max_speed_standard:
            self.max_speed *= 1.25
        
        if self.movement_mode == "Attack":
            if self.target_enemy is not None and self.target == self.box:
                self.follow_target_enemy(delta)
            else:
                self.follow_graph_path(delta)
        elif self.movement_mode == "Flee":
            # print(self.name + ". Target: " + str(self.target.idx) + ". Current box:" + str(self.box.idx) + ".")
            self.follow_graph_path(delta)

        self.update_heading()
        self.box = self.world.get_box_by_pos(int(self.pos.x), int(self.pos.y))

    def ally_attacking(self):
        for soldier in self.world.soldiers:
            if soldier is not self and soldier.movement_mode == "Attack":
                return True

        self.target_ally = None
        return False

    def choose_weapon(self):
        if self.movement_mode == 'Exchange Weapons':
            print("Can't choose weapon while exchanging weapons")
            return False

        if len(self.weapons) == 0:
            print("No weapons. Exchanging weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False

        weapon_0 = self.weapons[0]
        # weapon_1 = self.weapons[1]

        weapon_0_ammo = weapon_0.rounds_left_in_magazine + weapon_0.magazine_size * weapon_0.magazines_left 
        # weapon_1_ammo = weapon_1.rounds_left_in_magazine + weapon_1.magazine_size * weapon_1.magazines_left

        weapon_0_avg_dmg = weapon_0.damage * weapon_0.damage_factor
        # weapon_1_avg_dmg = weapon_1.damage * weapon_1.damage_factor

        if self.target_enemy is not None:
            target_health = self.target_enemy.health

        # check if out of ammo
        if weapon_0_ammo <= 0: # and weapon_1_ammo <= 0:
            print("No ammo. Changing weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False
        # check if attacking or patrolling and have insufficient ammo to kill the target
        elif self.target_enemy is not None and (self.movement_mode == 'Attack' or self.movement_mode == 'Patrol') and weapon_0_avg_dmg * weapon_0_ammo < target_health: # + weapon_1_avg_dmg * weapon_1_ammo < target_health:
            print("Insufficient ammo to kill target. Changing weapons. Rounds / Magazine Size / Magazines: " + str(weapon_0.rounds_left_in_magazine) + "/" + str(weapon_0.magazine_size) + "/" + str(weapon_0.magazines_left) + ". Avg Total Damage / Enemy Health: " + str(weapon_0_avg_dmg * weapon_0_ammo) + "/" + str(target_health) + ".")
            self.movement_mode = 'Exchange Weapons'
            return False
        # check if patrolling and would theoretically have insufficient ammo to kill the target
        elif self.target_enemy == None and self.movement_mode == 'Patrol' and weapon_0_avg_dmg * weapon_0_ammo < self.start_health: # + weapon_1_avg_dmg * weapon_1_ammo < self.start_health:
            print("Theoretically insufficient ammo to kill target. Changing weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False            

        # # check if only current weapon is out of ammo
        # if weapon_0_ammo <= 0 and weapon_1_ammo > 0:
        #     self.next_weapon()
        # # check if both weapons have ammo, if both weapons' probable damage dealt (accounting for explosive splash damage and multiple shotgun pellets vs fixed damage rifle and hand gun bullets)
        # # would be sufficient to kill the target, and the next weapon deals less damage
        # elif self.target_enemy is not None and weapon_0_ammo > 0 < weapon_1_ammo and weapon_0_avg_dmg > weapon_1_avg_dmg > target_health:
        #     self.next_weapon()

        return True

    def scared(self):
        if self.movement_mode == "Flee":
            return True

        if self.last_fear_ping == None:
            self.last_fear_ping = datetime.now()

        if (datetime.now() - self.last_fear_ping).total_seconds() >= 1:
            self.last_fear_ping = datetime.now()

            if self.movement_mode == "Attack":
                closest = None
                closest_dist = 9999999999999999999999
                
                for soldier in self.world.soldiers:
                    dist = self.distance(soldier.pos)

                    if dist < closest_dist:
                        closest = soldier
                        closest_dist = dist

                if closest is not None:
                    self.fear += 1 * (self.awareness_radius / max(closest_dist, 0.001))
            elif self.movement_mode == "Stationary" and self.fear > 0:
                # print(self.name + " calming down")
                self.fear = max(0, self.fear - 5)

        if self.fear >= 50:
            if self.movement_mode is not "Panicking" and self.fear > randrange(0, 100):
                self.movement_mode = "Panicking"
                self.get_new_path()
                # print("Now fleeing")

            return True

        return False

    def squad_overwhelmed(self):
        if self.movement_mode == "Getting Reinforcements":
            return True

        if len(self.world.soldiers) >= len(self.world.bases):
            return False

        if len(self.world.soldiers) == 1:
            return True

        attacking_enemies = 0

        for agent in self.world.agents:
            if agent.agent_type is not self.agent_type and agent.target_enemy in self.world.soldiers:
                attacking_enemies += 1

        if attacking_enemies >= len(self.world.soldiers):
            return True

        return False

    def render(self, color=None):
        egi.set_stroke(2)

        try:
            if self.path is not None and self.world.cfg['PATH_ON']:
                egi.red_pen()
                egi.line_by_pos(self.pos, self.current_node_box.get_vc("agent.render(), line from agent to box").copy())
                path = self.path.path

                for i in range(1,len(path)):
                    egi.line_by_pos(self.world.boxes[path[i-1]].get_vc("agent.render(), line from this to box").copy(), self.world.boxes[path[i]].get_vc("agent.render(), line from box to this").copy())

                if self == self.world.soldiers[0]:
                    for i in range(0, len(path)):
                        egi.circle(self.world.boxes[path[i]].get_vc("agent.render(), soldier leader path").copy(), 3)
        except TypeError:
            print("TypeError in agent.render()")
        except:
            print("Non-TypeError Error in agent.render()")


        if self.hit_time is not None:
            egi.red_pen()
        elif self.agent_type == "soldier":
            egi.green_pen()
        elif self.agent_type == "fugitive":
            egi.orange_pen()
        else:
            egi.grey_pen()

        if self.agent_type == 'fugitive':
            # render fugitive
            egi.circle(self.pos, self.radius)
            egi.circle(self.pos, self.radius * 2 / 3)
            egi.circle(self.pos, self.radius / 3)
            egi.cross(self.pos, self.radius)

            # render weapon's effective range
            if self.world.show_weapon_range:
                egi.set_pen_color(name='AQUA')
                egi.circle(self.pos, self.weapons[0].effective_range)

            # render field of view
            if self.world.show_awareness_range:
                if self.movement_mode == 'Attack':
                    egi.red_pen()
                else:
                    egi.yellow_pen()

                egi.circle(self.awareness_pos, self.awareness_radius)
        elif self.agent_type == 'soldier':
            # render soldier
            egi.circle(self.pos, self.radius)

            # render weapon's effective range
            if self.world.show_weapon_range:
                egi.set_pen_color(name='AQUA')
                egi.circle(self.pos, self.weapons[0].effective_range)
            
            # render field of view
            if self.world.show_awareness_range:
                if self.movement_mode == 'Attack':
                    egi.red_pen()
                else:
                    egi.yellow_pen()

                egi.circle(self.awareness_pos, self.awareness_radius)
        else:
            egi.circle(self.pos, self.radius)

        hp = self.health / self.start_health

        if hp > 0.75:
            egi.green_pen()
        elif hp > 0.25:
            egi.orange_pen()
        else:
            egi.red_pen()

        bar_left = self.pos + Vector2D(-self.radius * hp, self.radius * (1.75 * self.world.scale_scalar))
        bar_right = self.pos + Vector2D(self.radius * hp, self.radius * (1.75 * self.world.scale_scalar))
        egi.line_by_pos(bar_left, bar_right)

        # render obstacle avoidance
        # if self.show_avoidance:
        #     if self.obst_detected:
        #         egi.set_pen_color(name='RED')
        #     else:
        #         egi.set_pen_color(name='LIGHT_BLUE')

        #     egi.circle(self.pos, self.avoid_radius)

        #     if self.sensor_obst_detected:
        #         egi.set_pen_color(name='RED')
        #     else:
        #         egi.set_pen_color(name='LIGHT_BLUE')

        #     egi.circle(self.sensor_pos, self.avoid_radius)

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

    # def arrive(self, target_pos, speed):
    #     ''' this behaviour is similar to seek() but it attempts to arrive at
    #         the target position with a zero velocity'''
    #     decel_rate = self.DECELERATION_SPEEDS[speed]
    #     to_target = target_pos - self.pos
    #     dist = to_target.length()
    #     if dist > 0:
    #         # calculate the speed required to reach the target given the
    #         # desired deceleration rate
    #         speed = dist / decel_rate
    #         # make sure the velocity does not exceed the max
    #         speed = min(speed, self.max_speed)
    #         # from here proceed just like Seek except we don't need to
    #         # normalize the to_target vector because we have already gone to the
    #         # trouble of calculating its length for dist.
    #         desired_vel = to_target * (speed / dist)
    #         return (desired_vel - self.vel)
    #     return Vector2D(0, 0)
    
    # def avoid(self, obj_pos):
    #     desired_vel = (self.pos - obj_pos).normalise() * self.max_speed
    #     return (desired_vel - self.vel)
    
    # def avoid_agents(self, agents):
    #     agts = agents.copy()

    #     if self in agts:
    #         agts.remove(self)

    #     return self.avoid_obstacles(agts)  

    # def avoid_obstacles(self, obstacles):
    #     closest_obst = None
    #     closest_dist = 9999999999999

    #     closest_obst_sns = None
    #     closest_dist_sns = 9999999999999

    #     result = Vector2D(0, 0)

    #     for obstacle in obstacles:
    #         dist_to_self = self.distance(obstacle.pos)
    #         dist_to_avoid = (obstacle.pos - self.sensor_pos).length()

    #         if dist_to_self < self.avoid_radius + obstacle.radius and dist_to_self < closest_dist:
    #             closest_obst = obstacle
    #             closest_dist = dist_to_self

    #         if dist_to_avoid < self.avoid_radius + obstacle.radius and dist_to_avoid < closest_dist_sns:
    #             closest_obst_sns = obstacle
    #             closest_dist_sns = dist_to_avoid

    #     if closest_obst is not None:
    #         self.obst_detected = True
    #         result += self.avoid(closest_obst.pos)

    #     if closest_obst_sns is not None:
    #         self.sensor_obst_detected = True
    #         result += self.avoid(closest_obst_sns.pos)

    #     return result

    # def avoid_projectiles(self, projectiles):
    #     def avoid_projectile(self, projectile):
    #         to_projectile = projectile.pos - self.pos
    #         desired_vel = to_projectile.perp() * self.max_speed
    #         # print('desired vel is ' + str(desired_vel))
    #         return (desired_vel - self.vel)

    #     # print('checking for projectiles')

    #     closest_proj = None
    #     closest_dist = 9999999999999

    #     closest_proj_sns = None
    #     closest_dist_sns = 9999999999999

    #     result = Vector2D(0,0)

    #     for projectile in projectiles:
    #         dist_to_self = self.distance(projectile.pos)
    #         dist_to_avoid = (projectile.pos - self.sensor_pos).length()

    #         if dist_to_self < self.avoid_radius + projectile.radius and dist_to_self < closest_dist:
    #             closest_proj = projectile
    #             closest_dist = dist_to_self

    #         if dist_to_avoid < self.avoid_radius + projectile.radius and dist_to_avoid < closest_dist_sns:
    #             closest_proj_sns = projectile
    #             closest_dist_sns = dist_to_avoid

    #     if closest_proj is not None:
    #         self.obst_detected = True
    #         result += avoid_projectile(self, closest_proj)
        
    #     if closest_proj_sns is not None:
    #         self.sensor_obst_detected = True
    #         result += avoid_projectile(self, closest_proj_sns)

    #     return result

    # def avoid_walls(self, walls):
    #     closest_wall = None
    #     closest_dist = 9999999999999

    #     closest_wall_sns = None
    #     closest_dist_sns = 9999999999999

    #     result = Vector2D(0, 0)

    #     for wall in walls:
    #         wall_pos = wall.get_pos(self.pos)
    #         dist_to_self = self.distance(wall_pos)
    #         dist_to_avoid = (wall_pos - self.sensor_pos).length()

    #         if dist_to_self < self.avoid_radius and dist_to_self < closest_dist:
    #             closest_wall = wall
    #             closest_dist = dist_to_self

    #         if dist_to_avoid < self.avoid_radius and dist_to_avoid < closest_dist_sns:
    #             closest_wall_sns = wall
    #             closest_dist_sns = dist_to_avoid

    #     if closest_wall is not None:
    #         self.obst_detected = True
    #         result += self.avoid(closest_wall.get_pos(self.pos))

    #     if closest_wall_sns is not None:
    #         self.sensor_obst_detected = True
    #         result += self.avoid(closest_wall_sns.get_pos(self.pos))

    #     return result

    # def flee(self, hunter_pos, delta):
    #     ''' move away from hunter position '''
    #     if self.distance(hunter_pos) > self.avoidance_range:
    #         return self.wander(delta)
    #     else:
    #         return self.avoid(hunter_pos)

    def follow_graph_path(self, delta):
        try:
            self.following_enemy = False

            if self.path is None or len(self.path.path) is 0 or self.current_node_pos is None or self.current_node_box is None:
                print("follow graph path called get new path")
                self.get_new_path()

            to_current_node = (self.current_node_pos - self.pos).normalise() * self.max_speed * delta

            # if to_current_node.length() > self.distance(self.current_node_pos):
            #     to_current_node = to_current_node.normalise() * self.distance(self.current_node_pos)

            pos = Vector2D(self.pos.x + (to_current_node.x * self.world.scale_vector.x), self.pos.y + (to_current_node.y * self.world.scale_vector.y))

            for wall in self.world.walls:
                # if too close to a wall, move away from the wall
                if (pos - wall.get_vc("agent.follow_graph_path() 1")).length() < self.radius + wall.radius:
                    away_from_wall = (self.pos - wall.get_vc("agent.follow_graph_path() 1")).normalise() * self.max_speed * delta
                    pos += away_from_wall

            # if too close to a higher-ranking ally, don't move
            if self.agent_type == "soldier" and self is not self.world.soldiers[0]:
                i = 0

                while i in range(0, len(self.world.soldiers)):
                    soldier = self.world.soldiers[i]

                    if soldier == self:
                        i = len(self.world.soldiers)
                    elif soldier.distance(pos) < self.radius + soldier.radius:
                        return
                    else:
                        i += 1
            elif self.agent_type == "fugitive" and self is not self.world.fugitives[0]:
                i = 0

                while i in range(0, len(self.world.fugitives)):
                    fugitive = self.world.fugitives[i]

                    if fugitive == self:
                        i = len(self.world.fugitives)
                    elif fugitive.distance(pos) < self.radius + fugitive.radius:
                        if self.movement_mode == "Flee":
                            # print("Fleeing fugitive calmed down by friend")
                            self.sit_still()
                        return
                    else:
                        i += 1
            
            self.pos = pos
            path = self.path.path

            if self.distance(self.current_node_pos) < self.radius * 0.5:
                if len(path) > 1:
                    path.remove(path[0])
                    self.last_node_box = self.current_node_box
                    self.last_new_node_time = datetime.now()
                    self.current_node_box = self.world.boxes[path[0]]
                    self.current_node_pos = self.current_node_box.get_vc("agent.follow_graph_path() 2").copy() 
                elif self.movement_mode == "Flee":
                    # print("Fleeing fugitive reached end of fleeing path. Sitting still.")
                    self.sit_still()
                elif self.movement_mode == "Scout":
                    self.sit_still()

                    if self == self.world.soldiers[0]:
                        self.finish_scouting()
                elif self.movement_mode == "Getting Reinforcements":
                    self.world.set_soldiers()
                    self.max_speed = self.max_speed_standard
                    self.movement_mode = "Patrol"
                    self.get_new_path()
                else:
                    self.get_new_path()
        except TypeError:
            print("TypeError exception in agent.follow_path()")
        except:
            print("Unknown exception in agent.follow_path()")

    # def follow_path(self):
    #     if self.path.current_pt() is self.path.end_pt():
    #         return self.arrive(self.path.current_pt(), "slow")
    #     else:
    #         dist = self.distance(self.path.current_pt())

    #         if self.distance(self.path.current_pt()) < self.waypoint_threshold:
    #             self.path.inc_current_pt()
            
    #         if self.distance(self.path.current_pt()) < self.waypoint_threshold * 3:
    #             return self.arrive(self.path.current_pt(), "slow")
    #         else:
    #             return self.seek(self.path.current_pt())

    def follow_target_enemy(self, delta):
        self.following_enemy = True
        to_target_enemy = (self.target_enemy.pos - self.pos).normalise() * self.max_speed * delta
        pos = Vector2D(self.pos.x + (to_target_enemy.x * self.world.scale_vector.x), self.pos.y + (to_target_enemy.y * self.world.scale_vector.y))

        for wall in self.world.walls:
            if (pos - wall.get_vc("agent.follow_target_enemy()")).length() < (self.radius + wall.radius) * 1.1:
                return

        self.pos = pos

    # def hide(self, hunter, hiding_spots, delta):
    #     self.best_hiding_spot = None        
        
    #     # if only one hiding spot, just pick that spot 
    #     if len(hiding_spots) == 1:            
    #         self.best_hiding_spot = hiding_spots[0]
    #     elif len(hiding_spots) > 1:  
    #         # go to best hiding spot
    #         self.best_hiding_spot = self.get_best_hiding_spot(hiding_spots, hunter)
        
    #     if self.best_hiding_spot is None:
    #         # default: panic and run away from a random hunter
    #         return self.flee(hunter.pos, delta)
    #     else:
    #         self.best_hiding_spot.best = True    
    #         return self.arrive(self.best_hiding_spot.pos, 'fast')          

    # def hunt(self, evader, delta):
    #     prioritise_visible = False
    #     prioritise_close = False

    #     hunting = []
    #     visible = []
    #     close = []

    #     if self.distance(evader.pos) < self.hunt_dist + evader.avoid_radius:
    #         close.append(evader)

    #     for marker in self.fov_markers:
    #         if evader.distance(marker) < evader.avoid_radius: #* dist_multiplier
    #             visible.append(evader)

    #     hunting = list(set(visible) and set(close))

    #     visible.sort(key=lambda x: x.distance(self.pos))

    #     if len(hunting) > 0:
    #         hunting.sort(key=lambda x: x.distance(self.pos))  
    #         return self.pursuit(hunting[0])
    #     elif prioritise_close and len(close) > 0:
    #         if len(close) > 1:
    #             close.sort(key=lambda x: x.distance(self.pos))
    #         return self.pursuit(close[0])
    #     elif prioritise_visible and len(visible) > 0:
    #         if len(visible) > 1:
    #             visible.sort(key=lambda x: x.distance(self.pos))
    #         return self.pursuit(visible[0])
    #     else:
    #         hunting = close + visible
    #         if len(hunting) == 0:
    #             return self.wander(delta)
    #         else:
    #             if len(hunting) > 1:
    #                 hunting.sort(key=lambda x: x.distance(self.pos))                  
    #             return self.pursuit(hunting[0])

    # def pace(self, delta):
    #     threshold = 10

    #     if self.current_pt is not None and self.next_pt is not None:
    #         if self.distance(self.current_pt) < threshold:
    #             temp = self.current_pt
    #             self.current_pt = self.next_pt
    #             self.next_pt = temp

    #         return self.arrive(self.current_pt, 'fast')
    #     else:
    #         return Vector2D()

    # def pursuit(self, evader):
    #     ''' this behaviour predicts where an agent will be in time T and seeks
    #         towards that point to intercept it. '''
    #     to_evader = evader.pos - self.pos
    #     relative_heading = self.heading.dot(evader.heading)

    #     if (to_evader.dot(self.heading) > 0) and (relative_heading < 0.95):
    #         return self.seek(evader.pos)

    #     future_time = to_evader.length()/(self.max_speed + evader.speed())
    #     #future_time += (1 - self.heading.dot(evader.vel)) * - self.side.length()
    #     future_pos = evader.pos + evader.vel * future_time
    #     return self.seek(future_pos)

    # def seek(self, target_pos):
    #     ''' move towards target position '''
    #     desired_vel = (target_pos - self.pos).normalise() * self.max_speed
    #     return (desired_vel - self.vel)

    def sit_still(self):
        # print("Sitting still.")
        if self.target in self.world.targets:
            self.world.targets.remove(self.target)

        if self.max_speed is not self.max_speed_standard:
            self.max_speed_standard = self.max_speed_standard

        self.target = None
        self.path = None
        # self.current_node_pos = None
        # self.current_node_box = None
        self.movement_mode = "Stationary"
        
        if self.agent_type == "fugitive":
            self.fear = 45
        # self.get_new_path()

        self.update_heading()

    def update_heading(self):
        if self.movement_mode == "Stationary":
            closest = None
            closest_dist = 9999999999999999999999

            for agent in self.world.agents:
                if agent.agent_type is not self.agent_type:
                    dist = self.distance(agent.pos)

                    if dist < closest_dist:
                        closest = agent
                        closest_dist = dist

            if closest is not None:
                self.heading = (closest.pos - self.pos).get_normalised()
                self.side = self.heading.perp()

        elif self.current_node_pos is not None and self.pos is not None:
            self.heading = (self.current_node_pos - self.pos).get_normalised()
            self.side = self.heading.perp()

    # def wander(self, delta):
    #     self.wandering = True
    #     ''' Random wandering using a projected jitter circle. '''
    #     wt = self.wander_target
    #     # this behaviour is dependant on the update rate, so this line must
    #     # be included when using time independent framerates.
    #     jitter_tts = self.wander_jitter * delta # this time slice
    #     # first, add a small random vector to the target's position
    #     wt += Vector2D(uniform(-1, 1) * jitter_tts, uniform(-1, 1) * jitter_tts)
    #     # re-project this new fector back onto a unit circle
    #     wt.normalise()
    #     # increase the length of the vector to the same as the radius
    #     # of the wander circle
    #     wt *= self.wander_radius
    #     # move the target into a position WanderDist in front of the agent
    #     target = wt + Vector2D(self.wander_dist, 0)
    #     # project the target into world space
    #     wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)
    #     # and steer towards it
    #     force = self.seek(wld_target)
    #     return force 

    # Marksmanship Methods ------------------------------------------------------------------------

    def aim_shot(self, target):
        # print("stationary: " + str(target.pos))

        if target.movement_mode == 'Stationary':
            target_pos = target.pos.copy()
        else:
            ''' this behaviour predicts where an agent will be in time T and seeks
            towards that point to intercept it. '''
            to_target = target.pos - self.pos
            future_target_pos = target.pos.copy()
            dist = to_target.length()
            loop = True
            loop_count = 0

            target_path_measurements = self.get_target_path_measurements(self.target_enemy)

            # loops its predictive logic several times to get progressively better predictions. If it would
            # get to an iteration where the distance between the projectile and target would be worse than
            # on the last iteration, it uses the last iteration
            while loop:
                loop_count += 1
                future_time = (future_target_pos - self.pos).length()/(self.weapons[0].speed)        # first loop: current target.pos
                
                ### get_future_pos_with_accel() now useless as there is no vel or accel. Need to calculate position based on current vector
                #   ((current target node pos - current target pos).get_normalised()) and / or path . . . calculate length of path using 
                #   distance from target to target's current target node, plus distance between each node in the path, predict where along
                #   the path the target will be at a particular time, and go from there. If projection goes beyond end point of the path, 
                #   assume the target will continue on the same heading.
                # future_target_pos = target.pos + self.get_future_pos_with_accel(target.vel, target.accel, future_time)
                future_target_pos = self.get_future_pos_on_path(self.target_enemy, target_path_measurements, self.target_enemy.max_speed, future_time)
                # self.world.wrap_around(future_target_pos)

                vel_to_future_pos = (future_target_pos - self.pos).normalise() * self.weapons[0].speed
                future_self_pos = self.pos + vel_to_future_pos * future_time
                new_dist = (future_target_pos - future_self_pos).length()

                # keep iterating?
                if new_dist < dist: 
                    dist = new_dist

                    # is it's predicted pos close enough?
                    if dist < self.target_enemy.radius * 0.1:
                        # print('dist between predicted positions less than 0.1 times the targets radius')
                        loop = False

                else:
                    loop = False

            target_pos = future_target_pos
            # print("predictive: " + str(target_pos))
            # print("loop count: " + str(loop_count))

        if self.distance(target_pos) <= self.weapons[0].effective_range + self.target_enemy.radius:
            return target_pos
        else:
            return None

    def get_future_pos_with_accel(self, start_vel, accel, time):
        displacement = (start_vel * time) + (0.5 * accel * time * time) #d = ut + 0.5at^2
        return displacement 

    def get_target_path_measurements(self, target):
        result = {}
        total_dist = 0

        boxes = self.world.boxes

        # target to current node
        if target.current_node_box is not None:
            dist = (target.current_node_box.get_vc("agent.get_target_path_measurements(), target to current node") - target.pos).length()
        else:
            return None

        total_dist += dist
        result[0] = {"box": target.current_node_box, "dist": total_dist}

        if target.path is not None and len(target.path.path) > 0:
            path = target.path.path

            # current node to first node in path
            dist = (boxes[path[0]].get_vc("agent.get_target_path_measurements(), current node to first node in path, path node") - target.current_node_box.get_vc("agent.get_target_path_measurements(), current node to first node in path, current node")).length()
            total_dist += dist
            result[1] = {"box": boxes[path[0]], "dist": total_dist}

            if len(path) > 1:
                # first node in path to last node in path
                for i in range(1, len(path)):
                    dist = (boxes[path[i]].get_vc("agent.get_target_path_measurements(), path nodes, " + str(i)) - boxes[path[i-1]].get_vc("agent.get_target_path_measurements(), path nodes, " + str(i-1))).length()
                    total_dist += dist
                    result[i+1] = {"box": boxes[path[i]], "dist": total_dist}

        return result

    def get_future_pos_on_path(self, target, path_measurements, speed, time):
        pos = Vector2D()
        dist = speed * time
        
        if path_measurements is not None:
            i = len(path_measurements) - 1
        
            while i >= 0:
                if dist > path_measurements[i]["dist"]:
                    pos = path_measurements[i]["box"].get_vc("agent.get_future_pos_on_path()").copy()
                    dif = dist - path_measurements[i]["dist"]
                    pos += target.heading * dif

                    return pos

                i -= 1

        return target.pos + (target.heading * dist)

    def shoot(self, target_pos):
        if self.agent_type == "fugitive":
            print(self.name + " shooting")
        original_target_pos = target_pos.copy()

        if self.weapons[0].name == 'Shotgun':
            # shot gun scatters multiple projectiles
            loop = 5
        else:
            # everything else fires one projectile at a time
            loop = 1

        while loop > 0:
            # affect accuracy if using an inaccurate weapon
            if self.weapons[0].accuracy_modifier > 0:
                target_pos.x = int(randrange(int(original_target_pos.x - self.weapons[0].accuracy_modifier), int(original_target_pos.x + self.weapons[0].accuracy_modifier)))
                target_pos.y = int(randrange(int(original_target_pos.y - self.weapons[0].accuracy_modifier), int(original_target_pos.y + self.weapons[0].accuracy_modifier)))

            # calculate velocity
            p_heading = (target_pos - self.pos).normalise()
            p_vel = p_heading * self.weapons[0].speed

            # set up and shoot projectile
            p = self.weapons[0].projectile_pool[0]
            self.weapons[0].projectile_pool.remove(p)
            p.vel = p_vel
            p.pos = self.pos.copy()
            # p.p_type = self.weapon
            p.damage = self.weapons[0].damage

            # where's the grenade gonna land?
            if self.weapons[0].name == 'Hand Grenade':
                p.target = target_pos

            # add new projectile
            self.world.projectiles.append(p)
            p.owner_on_firing = self

            loop -= 1

        # self.hunger += self.weapons[0].stamina_drain

    # Additional assistive methods used by Agent --------------------------------------------------

    # def collided(self):
    #     if self.world.obstacles_enabled:
    #         for obstacle in self.world.obstacles:
    #             if self.distance(obstacle.pos) < self.radius + obstacle.radius:
    #                 return True

    #     if self.world.walls_enabled:
    #         for wall in self.world.walls:
    #             if wall.distance(self.pos) < self.radius:
    #                 return True

    #     for agent in self.world.agents:
    #         if agent is not self and self.distance(agent.pos) < self.radius + agent.radius:
    #             return True

    #     return False

    def distance(self, target_pos):
        to_target = target_pos - self.pos
        dist = to_target.length()
        return dist

    def finish_scouting(self):
        self.movement_mode = "Patrol"
        self.get_new_path()

        for soldier in self.world.soldiers:
            if soldier is not self:
                soldier.movement_mode = "Patrol"
                soldier.get_new_path()

    # def get_best_hiding_spot(self, spots, hunter):
    #     # sort and / or rank the hiding spots
    #     prioritise_close = False
    #     prioritise_far_from_hunter = False

    #     for spot in spots:
    #         spot.rank = 0

    #     if prioritise_close:
    #         # get closest
    #         for spot in spots:
    #             spot.dist_to_evader = self.distance(spot.pos)                
    #         spots.sort(key=lambda x: x.dist_to_evader)
    #     elif prioritise_far_from_hunter:
    #         # get furthest from hunter
    #         spots.sort(key=lambda x: x.avg_dist_to_hunter, reverse=True)
    #     else:
    #         # sort and increment spots' rank by closeness, lowest (index = 0) to highest (index = len - 1)
    #         for spot in spots:
    #             spot.dist_to_evader = self.distance(spot.pos)
    #         spots.sort(key=lambda x: x.dist_to_evader)
    #         for i in range(len(spots) - 1):
    #             spots[i].rank += i

    #         # sort and increment spots' rank by distance from hunter, highest (index = 0) to lowest (index = len - 1)
    #         spots.sort(key=lambda x: x.avg_dist_to_hunter, reverse=True)
    #         for i in range(len(spots) - 1):
    #             spots[i].rank += i

    #         # increment spots' rank if moving to them would require crossing the hunter's line of sight
    #         for spot in spots:
    #             if self.intersect(self.pos, spot.pos, hunter.fov_markers[0], hunter.fov_markers[1]) or self.intersect(self.pos, spot.pos, hunter.fov_markers[2], hunter.fov_markers[3]) or self.intersect(self.pos, spot.pos, hunter.fov_markers[4], hunter.fov_markers[5]):
    #                 spot.rank += 999999999

    #         # sort spots by rank, lowest (index = 0) to highest (index = len - 1) 
    #         spots.sort(key=lambda x: x.rank)

    #     for i in range(0, len(spots) - 1):
    #         if spots[i].valid:
    #             return spots[i]

    #     return None

    def get_new_path(self):
        if self.target in self.world.targets:
            self.world.targets.remove(self.target)

        self.last_new_node_time = datetime.now()

        if self.agent_type == "soldier":
            if self.movement_mode == "Stationary":
                return
            elif self.movement_mode == "Getting Reinforcements":
                print(self.name + " getting reinforcements path")
                self.target = self.world.bases[0]
            elif self.movement_mode == "Attack":
                if self.target_enemy is not None:
                    print(self.name + " getting attacking path")
                    self.target = self.target_enemy.box
                else:
                    self.movement_mode = "Patrol"
            elif self.movement_mode == "Assist":
                if self.target_ally is not None:
                    print(self.name + " getting assistive path")
                    self.target = self.target_ally.box
                else:
                    self.movement_mode = "Patrol"
            elif self.movement_mode == "Scout":
                print(self.name + " getting scouting path")
                target = self.world.boxes[randrange(0, len(self.world.boxes))]

                while not self.suitable_scouting_location(target):
                    target = self.world.boxes[randrange(0, len(self.world.boxes))]
                   
                self.target = target

            if self.movement_mode == "Patrol":
                if self == self.world.soldiers[0]:
                    print(self.name + " getting commander patrol path")
                    if self.box.waypoint is not None:
                        self.world.update_waypoint(self.box.waypoint)
                    self.target = self.world.get_current_waypoint_node()
                else:
                    print(self.name + " getting trooper patrol path")
                    self.target = self.world.soldiers[0].target
        elif self.agent_type == "fugitive":
            if self.movement_mode == "Attack":
                if self.target_enemy is not None:
                    print(self.name + " getting attacking path")
                    self.target = self.target_enemy.box
                else:
                    self.movement_mode = "Stationary"
            elif self.movement_mode == "Panicking":
                print(self.name + " getting fleeing path")

                if len(self.world.soldiers) == 0:
                    self.movement_mode = "Stationary"
                    return

                target = self.world.boxes[randrange(0, len(self.world.boxes))]

                while not self.suitable_fleeing_location(target):
                    if len(self.world.soldiers) == 0:
                        self.movement_mode = "Stationary"
                        return

                    target = self.world.boxes[randrange(0, len(self.world.boxes))]
                   
                self.target = target
                self.movement_mode = "Flee"
            elif self.movement_mode == "Flee":
                print(self.name + " finished fleeing. Now sitting still.")
                self.sit_still()
                return
            elif self.movement_mode == "Stationary":
                # print("fugitive is stationary")
                return
        else:
            print("Error: Invalid agent type submitted to agent.get_new_path(). " + self.name + " defaulting to wandering.")
            target = self.world.boxes[randrange(0, len(self.world.boxes))]

            # while target.kind == "X" or target == self.world.get_box_by_pos(int(self.pos.x), int(self.pos.y)):
            while target.kind == "X" or target == self.box:
                target = self.world.boxes[randrange(0, len(self.world.boxes))]
               
            self.target = target

        self.world.targets.append(self.target)
        self.plan_path(search_modes[self.world.window.search_mode], self.world.window.limit)

    # def get_path_length(self, path):
    #     total_dist = 0
    #     boxes = self.world.boxes

    #     if path is not None:
    #         # current node to first node in path
    #         dist = self.distance(boxes[path[0]].get_vc("agent.get_path_length(), self to first node in path, path node"))
    #         total_dist += dist

    #         # first node in path to last node in path
    #         for i in range(1, len(path)):
    #             dist = (boxes[path[i]].get_vc("agent.get_path_length(), path nodes, " + str(i)) - boxes[path[i-1]].get_vc("agent.get_path_length(), path nodes, " + str(i-1))).length()
    #             total_dist += dist

    #     return total_dist

    # def update_hunt_dist(self):
    #     dist_multiplier = 1.25

    #     if self.hunt_dist < self.avoid_radius * 2 * dist_multiplier - self.avoid_radius:    # equivalent to (self.avoid_radius + evader.avoid_radius) * dist_multiplier - evader.avoid_radius as self and evader should have the same avoid_radius
    #         self.hunt_dist = self.avoid_radius * 2 * dist_multiplier - self.avoid_radius

    # def get_random_valid_position(self, max_x, max_y, obstacles, agents):
    #     valid = False
    #     pos = Vector2D()

    #     while not valid:
    #         valid = True
    #         pos = Vector2D(randrange(max_x), randrange(max_y))
            
    #         for obstacle in obstacles:
    #             if obstacle.distance(pos) <= self.avoid_radius + obstacle.radius:
    #                 valid = False

    #         for agent in agents:
    #             if agent is not self and agent.distance(pos) <= self.avoid_radius + agent.avoid_radius:
    #                 valid = False

    #     return pos

    # The main function that returns true if line segment 'p1q1' 
    # and 'p2q2' intersect. 
    # Borrowed from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    # def intersect(self, p1, q1, p2, q2): 
    #     # Given three colinear points p, q, r, the function checks if 
    #     # point q lies on line segment 'pr' 
    #     def on_segment(p, q, r): 
    #         if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)): 
    #            return True 
          
    #         return False       
      
    #     # To find orientation of ordered triplet (p, q, r). 
    #     # The function returns following values 
    #     # 0 --> p, q and r are colinear 
    #     # 1 --> Clockwise 
    #     # 2 --> Counterclockwise 
    #     def orientation(p, q, r): 
    #         val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
          
    #         if val == 0:
    #             return 0    # colinear 
    #         elif val > 0:
    #             return 1    # clockwise 
    #         else:
    #             return 2    # anticlockwise

    #     # Find the four orientations needed for general and 
    #     # special cases 
    #     o1 = orientation(p1, q1, p2) 
    #     o2 = orientation(p1, q1, q2) 
    #     o3 = orientation(p2, q2, p1)
    #     o4 = orientation(p2, q2, q1)
      
    #     # General case 
    #     if (o1 is not o2 and o3 is not o4): 
    #         return True 
      
    #     # Special Cases 
    #     # p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    #     if o1 == 0 and on_segment(p1, p2, q1):
    #         return True
      
    #     # p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    #     if o2 == 0 and on_segment(p1, q2, q1):
    #         return True
      
    #     # p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    #     if o3 == 0 and on_segment(p2, p1, q2):
    #         return True
      
    #     # p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    #     if o4 == 0 and on_segment(p2, q1, q2):
    #         return True
      
    #     return False # Doesn't fall in any of the above cases 

    def see_target(self):
        for agent in self.world.agents:
            if agent.agent_type is not self.agent_type and self.target_and_path_in_range(agent):
                return True

        return False

    def calculate_path(self, search, target_box, limit):
        try:
            cls = SEARCHES[search]

            if target_box == None:
                return None
            else:
                return cls(self.world.graph, self.box.idx, target_box.idx, limit)
        except:
            print("Error in agent.calculate_path()")

    def plan_path(self, search, limit):
        '''Conduct a nav-graph search from the current world start node to the
        current target node, using a search method that matches the string
        specified in `search`.
        '''
        try:
            cls = SEARCHES[search]
            self.path = cls(self.world.graph, self.box.idx, self.target.idx, limit)

            if len(self.path.path) > 0:
                path = self.path.path
                boxes = self.world.boxes
                self.last_node_box = self.current_node_box
                self.current_node_box = self.world.boxes[path[0]]
                self.current_node_pos = self.current_node_box.get_vc("agent.plan_path() 1").copy()

                if len(self.path.path) > 1 and self.distance(boxes[path[1]].get_vc("agent.plan_path() 2").copy()) < (boxes[path[0]].get_vc("agent.plan_path() 3").copy() - boxes[path[1]].get_vc("agent.plan_path() 4").copy()).length():
                    path.remove(path[0])
                    self.last_node_box = self.current_node_box
                    self.last_new_node_time = datetime.now()
                    self.current_node_box = boxes[path[0]]
                    self.current_node_pos = self.current_node_box.get_vc("agent.follow_graph_path()").copy() 
        except:
            print("Error in running agent.plan_path()")

    def position_in_random_box(self):
        self.box = self.world.boxes[randrange(0, len(self.world.boxes))]

        while self.box.kind == "X":
            self.box = self.world.boxes[randrange(0, len(self.world.boxes))]
        
        self.pos = self.box.get_vc("agent.position_in_random_box()").copy()

    # def randomise_path(self):
    #     num_pts = 4
    #     cx = self.world.cx
    #     cy = self.world.cy
    #     margin = min(cx, cy) * (1/6)    #use this for padding in the next line
    #     self.path.create_random_path(num_pts, margin, margin, cx - margin, cy - margin)

    # def speed(self):
    #     return self.vel.length()

    def suitable_fleeing_location(self, target):
        try:

            if target.kind == "X":
                return False

            for agent in self.world.agents:
                if agent.agent_type == "soldier":
                    if (target.get_vc("agent.suitable_fleeing_location()") - agent.awareness_pos).length() < agent.awareness_radius * 3:
                        return False
                else:
                    if target == agent.box:
                        return False

            return True
        except:
            print("Error in agent.suitable_fleeing_location()")

    def suitable_scouting_location(self, target):
        if target.kind == "X":
            return False

        max_range = self.world.soldiers[0].awareness_radius * 1.5

        if (self.world.soldiers[0].awareness_pos - target.get_vc("agent.suitable_scouting_location(), 1")).length() > max_range:
            return False

        path_to_target = self.calculate_path(search_modes[self.world.window.search_mode], target, self.world.window.limit)

        if path_to_target is None:
            return False

        if self.distance(self.world.soldiers[0].pos) > max_range:
            self_in_range = False
        else:
            self_in_range = True

        for box_index in path_to_target.path:
            if (self.awareness_pos - self.world.boxes[box_index].get_vc("agent.suitable_scouting_location(), 2")).length() > max_range:
                if not self_in_range:
                    self_in_range = self_in_range
                else:
                    return False

        return True

    def target_and_path_in_range(self, target):
        if target.distance(self.awareness_pos) >= self.awareness_radius + target.radius:
            return False

        path_to_target = self.calculate_path(search_modes[self.world.window.search_mode], target.box, self.world.window.limit)

        if path_to_target is None:
            return False

        for box_index in path_to_target.path:
            if (self.awareness_pos - self.world.boxes[box_index].get_vc("agent.target_and_path_in_range()")).length() >= self.awareness_radius + target.radius:
                return False

        return True

    # def update_fov(self, walls, agents):
    #     crossed_obj = False
    #     max_fov_length = self.avoid_radius * 20
    #     fov_length = self.radius
    #     fovm = Vector2D()
    #     fov_marker_left = Vector2D()
    #     fov_marker = Vector2D()
    #     fov_marker = Vector2D()
    #     markers = []

    #     offset = self.radius # 5 * self.world.scalar_scale

    #     agents = agents.copy()

    #     if self in agents:
    #         agents.remove(self)

    #     while not crossed_obj and fov_length < max_fov_length: 
    #         fovm = Vector2D(fov_length, -offset)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, -offset * 2 / 3)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, -offset / 3)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, 0)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, offset / 3)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, offset * 2 / 3)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #         fovm = Vector2D(fov_length, offset)
    #         markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
            
    #         for box in walls:
    #             for marker in markers:
    #                 box = self.world.get_box_by_pos(int(marker.x), int(marker.y))

    #                 if (marker - box._vc).length() < box.radius:
    #                     crossed_obj = True
    #                     overshoot = box.radius - (marker - box._vc).length()

    #         if not crossed_obj:
    #             for a in agents:
    #                 for marker in markers:
    #                     if a.distance(marker) < a.radius:
    #                         crossed_obj = True
    #                         overshoot = a.radius - a.distance(fov_marker)

    #                         if a in self.world.targets:
    #                             self.see_target = True

    #         if not crossed_obj:
    #             fov_length += self.radius * 0.5
    #         else:
    #             fov_length -= overshoot

    #     markers = []
    #     fov_length = min(fov_length, max_fov_length)
    #     fovm = Vector2D(fov_length, 0)
    #     left = Vector2D(0, -self.hunt_dist)
    #     m_left = Vector2D(fov_length, -offset)
    #     right = Vector2D (0, self.hunt_dist)
    #     m_right = Vector2D(fov_length, offset)
    #     markers.append(self.world.transform_point(left, self.pos, self.heading, self.side))
    #     markers.append(self.world.transform_point(m_left, self.pos, self.heading, self.side))
    #     markers.append(self.pos)
    #     markers.append(self.world.transform_point(fovm, self.pos, self.heading, self.side))
    #     markers.append(self.world.transform_point(right, self.pos, self.heading, self.side))
    #     markers.append(self.world.transform_point(m_right, self.pos, self.heading, self.side))
    #     self.fov_markers = markers
        
