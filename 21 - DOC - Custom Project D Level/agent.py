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

class Agent(object):
    # Agent Setup ---------------------------------------------------------------------------------

    def __init__(self, world=None, scale=30.0, agent_type="fugitive", movement_mode=None, combat_mode=None, weapons=[], radius=6.0, box=None, path=None, name="Agent", respawnable=False):
        self.world = world
        self.name = name
        self.agent_type = agent_type
        self.movement_mode = movement_mode
        self.combat_mode = combat_mode
        self.weapons = weapons

        self.health = 1000
        self.start_health = 1000
        self.respawnable = respawnable

        # what am I and where am I going?
        dir = radians(random()*360)
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.radius = radius
        self.radius_standard = radius
        self.max_speed_standard = 80
        self.max_speed = 80

        # where am i?
        if box == None:
            self.pos = None
            self.box = None
            self.position_in_random_box()
        else:
            self.box = self.world.boxes[box]
            self.pos = self.box.get_vc("agent init")

        self.hit_time = None
        self.last_shot = datetime.now()

        self.fear = 0
        self.last_fear_ping = None

        self.path = path
        self.last_box = None
        self.last_node_box = None
        self.current_node_box = None
        self.current_node_pos = None
        self.last_new_node_time = None

        self.awareness_radius_standard = 100
        self.awareness_radius = 100
        self.awareness_pos = self.world.transform_point(Vector2D(self.awareness_radius * 0.625, 0), self.pos, self.heading, self.side)

        self.target = None
        self.target_enemy = None
        self.following_enemy = False
        self.target_ally = None

        self.world.change_weapons(self)

    # The central logic of the Agent class --------------------------------------------------------

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
        if self.health <= 0:
            print(self.name + ": x_x")
            self.world.destroy_soldier(self)
            return
       
        self.select_soldier_movement_mode()
        self.handle_weapons()
        self.move_soldier(delta)

    def select_soldier_movement_mode(self):
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

    def ally_attacking(self):
        for soldier in self.world.soldiers:
            if soldier is not self and soldier.movement_mode == "Attack":
                return True

        self.target_ally = None
        return False

    def move_soldier(self, delta):
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
        box = self.world.get_box_by_pos(int(self.pos.x), int(self.pos.y))
        
        if box is not self.box:
            self.last_box = self.box
            self.box = box

            if self.world.soldiers[0] == self and box.waypoint is not None and self.last_box not in self.world.waypoints[box.waypoint].nodes and self.movement_mode is not "Patrol":
                self.world.update_waypoint(box.waypoint)

        self.awareness_pos = Vector2D(self.awareness_radius * 0.625, 0)
        self.awareness_pos = self.world.transform_point(self.awareness_pos, self.pos, self.heading, self.side)

    def update_fugitive(self, delta):
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

        if not self.scared():
            self.select_fugitive_movement_mode()
            self.handle_weapons()            
        elif self.max_speed == self.max_speed_standard:
            self.max_speed *= 1.25
        
        self.move_fugitive(delta)

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
                self.fear = max(0, self.fear - 5)

        if self.fear >= 50:
            if self.movement_mode is not "Panicking" and self.fear > randrange(0, 100):
                self.movement_mode = "Panicking"
                self.get_new_path()

            return True

        return False

    def select_fugitive_movement_mode(self):
        if self.see_target():
            self.movement_mode = "Attack"

            if self.target_enemy == None or self.target_enemy.distance(self.pos) > self.awareness_radius + self.target_enemy.radius:
                self.target_enemy = None

                for agent in self.world.agents:
                    if agent.agent_type is not self.agent_type and (self.target_enemy == None or self.distance(self.target_enemy.pos) > self.distance(agent.pos)) and self.target_and_path_in_range(agent):
                        self.target_enemy = agent

            if self.target_enemy is not None and self.target_enemy.box is not self.target:
                self.get_new_path()
        elif self.movement_mode is not "Stationary":
            self.sit_still()

    def move_fugitive(self, delta):
        if self.movement_mode == "Attack":
            if self.target_enemy is not None and self.target == self.box:
                self.follow_target_enemy(delta)
            else:
                self.follow_graph_path(delta)
        elif self.movement_mode == "Flee":
            self.follow_graph_path(delta)

        self.update_heading()
        self.box = self.world.get_box_by_pos(int(self.pos.x), int(self.pos.y))

        self.awareness_pos = Vector2D(self.awareness_radius * 0.625, 0)
        self.awareness_pos = self.world.transform_point(self.awareness_pos, self.pos, self.heading, self.side)

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
            if self.world.show_fugitive_awareness_range:
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
            if self.world.show_soldier_awareness_range:
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

    # The motion behaviours of Agent --------------------------------------------------------------

    def follow_graph_path(self, delta):
        try:
            self.following_enemy = False

            if self.path is None or len(self.path.path) is 0 or self.current_node_pos is None or self.current_node_box is None:
                print("follow graph path called get new path")
                self.get_new_path()

            to_current_node = (self.current_node_pos - self.pos).normalise() * self.max_speed * delta
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

    def follow_target_enemy(self, delta):
        self.following_enemy = True
        to_target_enemy = (self.target_enemy.pos - self.pos).normalise() * self.max_speed * delta
        pos = Vector2D(self.pos.x + (to_target_enemy.x * self.world.scale_vector.x), self.pos.y + (to_target_enemy.y * self.world.scale_vector.y))

        for wall in self.world.walls:
            if (pos - wall.get_vc("agent.follow_target_enemy()")).length() < (self.radius + wall.radius) * 1.1:
                return

        self.pos = pos

    def sit_still(self):
        # print("Sitting still.")
        if self.target in self.world.targets:
            self.world.targets.remove(self.target)

        if self.max_speed is not self.max_speed_standard:
            self.max_speed = self.max_speed_standard

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

    # Marksmanship Methods ------------------------------------------------------------------------
    
    def handle_weapons(self):
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
                elif self.movement_mode == 'Attack' or self.movement_mode == "Flee" or self.movement_mode == "Getting Reinforcements":
                    self.combat_mode = 'Aiming'
                else:
                    self.combat_mode = 'Ready'

            # aim and shoot
            if self.combat_mode == 'Aiming':
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

    def choose_weapon(self):
        if self.movement_mode == 'Exchange Weapons':
            print("Can't choose weapon while exchanging weapons")
            return False

        if len(self.weapons) == 0:
            print("No weapons. Exchanging weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False

        weapon_0 = self.weapons[0]
        weapon_0_ammo = weapon_0.rounds_left_in_magazine + weapon_0.magazine_size * weapon_0.magazines_left
        weapon_0_avg_dmg = weapon_0.damage * weapon_0.damage_factor

        if self.target_enemy is not None:
            target_health = self.target_enemy.health

        # check if out of ammo
        if weapon_0_ammo <= 0:
            print("No ammo. Changing weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False
        # check if attacking or patrolling and have insufficient ammo to kill the target
        elif self.target_enemy is not None and weapon_0_avg_dmg * weapon_0_ammo < target_health:
            print("Insufficient ammo to kill target. Changing weapons. Rounds / Magazine Size / Magazines: " + str(weapon_0.rounds_left_in_magazine) + "/" + str(weapon_0.magazine_size) + "/" + str(weapon_0.magazines_left) + ". Avg Total Damage / Enemy Health: " + str(weapon_0_avg_dmg * weapon_0_ammo) + "/" + str(target_health) + ".")
            self.movement_mode = 'Exchange Weapons'
            return False
        # check if patrolling and would theoretically have insufficient ammo to kill the target
        elif self.target_enemy == None and weapon_0_avg_dmg * weapon_0_ammo < self.start_health:
            print("Theoretically insufficient ammo to kill target. Changing weapons.")
            self.movement_mode = 'Exchange Weapons'
            return False            

        return True 

    def aim_shot(self, target):
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
                future_target_pos = self.get_future_pos_on_path(self.target_enemy, target_path_measurements, self.target_enemy.max_speed, future_time)

                vel_to_future_pos = (future_target_pos - self.pos).normalise() * self.weapons[0].speed
                future_self_pos = self.pos + vel_to_future_pos * future_time
                new_dist = (future_target_pos - future_self_pos).length()

                # keep iterating?
                if new_dist < dist: 
                    dist = new_dist

                    # is it's predicted pos close enough?
                    if dist < self.target_enemy.radius * 0.1:
                        loop = False

                else:
                    loop = False

            target_pos = future_target_pos

        if self.distance(target_pos) <= self.weapons[0].effective_range + self.target_enemy.radius:
            return target_pos
        else:
            return None

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
    
    def calculate_path(self, search, target_box, limit):
        try:
            cls = SEARCHES[search]

            if target_box == None:
                return None
            else:
                return cls(self.world.graph, self.box.idx, target_box.idx, limit)
        except:
            print("Error in agent.calculate_path()")

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
                return
        else:
            print("Error: Invalid agent type submitted to agent.get_new_path(). " + self.name + " defaulting to wandering.")
            target = self.world.boxes[randrange(0, len(self.world.boxes))]

            while target.kind == "X" or target == self.box:
                target = self.world.boxes[randrange(0, len(self.world.boxes))]
               
            self.target = target

        self.world.targets.append(self.target)
        self.plan_path(search_modes[self.world.window.search_mode], self.world.window.limit)

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

    def see_target(self):
        for agent in self.world.agents:
            if agent.agent_type is not self.agent_type and self.target_and_path_in_range(agent):
                return True

        return False

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
