'''A 2d world that supports agents with steering behaviour

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from matrix33 import Matrix33
from graphics import egi
from agent import Agent
from path import Path
from random import random, randrange, uniform


class World(object):
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.wall_margin = 5

        self.agents = []
        self.target = None
        self.shooter = None
        self.weapons = []
        self.projectiles = []
        self.obstacles = []
        self.obstacles_enabled = False
        self.walls = []
        self.walls_enabled = False
        self.paused = True
        self.showinfo = True
        self.hiding_spots = []
        self.agent_scale = 30.0
        self.agent_radius = 1.0 * self.agent_scale
        self.agent_avoid_radius = 2 * self.agent_radius

        self.ammo_station = Vector2D(cx * 0.05, cy * 0.9)
        self.food_station = Vector2D(cx * 0.95, cy * 0.9)
        self.station_size = 30

    def update(self, delta):
        if not self.paused:
            # if self.target.movement_mode == 'Escape':
            #     self.hiding_spots = self.get_hiding_spots(self.shooter, self.obstacles)

            for agent in self.agents:
                agent.update(delta)

            for projectile in self.projectiles:
                projectile.update(delta)

    def render(self):
        egi.green_pen()

        egi.closed_shape([
            Vector2D(self.ammo_station.x - 15, self.ammo_station.y + 15),
            Vector2D(self.ammo_station.x + 15, self.ammo_station.y + 15),
            Vector2D(self.ammo_station.x + 15, self.ammo_station.y - 15),
            Vector2D(self.ammo_station.x - 15, self.ammo_station.y - 15)
        ])

        egi.text_at_pos(self.ammo_station.x - 4, self.ammo_station.y - 8, 'A')

        egi.closed_shape([
            Vector2D(self.food_station.x - 15, self.food_station.y + 15),
            Vector2D(self.food_station.x + 15, self.food_station.y + 15),
            Vector2D(self.food_station.x + 15, self.food_station.y - 15),
            Vector2D(self.food_station.x - 15, self.food_station.y - 15)
        ])

        egi.text_at_pos(self.food_station.x - 4, self.food_station.y - 8, 'F')

        for agent in self.agents:
            agent.render()

        if self.obstacles_enabled:
            for obstacle in self.obstacles:
                obstacle.render()

        if self.walls_enabled:
            for wall in self.walls:
                wall.render()

        for projectile in self.projectiles:
            projectile.render()

        if self.showinfo:
            infotext = ', '.join(set(agent.agent_type for agent in self.agents))
            egi.white_pen()

            if self.shooter is not None and self.shooter.ready:
                health_status = 'Soldier: ' + str(self.shooter.health) + ' HP. Hunger: ' + str(int(self.shooter.hunger)) + '/50. '
                agent_status = 'Soldier Status: ' + self.shooter.movement_mode + ', ' + self.shooter.combat_mode + '. ' 
                weapon_0 = self.shooter.weapons[0]
                weapon_1 = self.shooter.weapons[1]
                agent_status = agent_status + weapon_0.name + ' [' + str(weapon_0.rounds_left_in_magazine) + '/' + str(weapon_0.magazine_size) + '/' + str(weapon_0.magazines_left) +'] / ' + weapon_1.name + ' [' + str(weapon_1.rounds_left_in_magazine) + '/' + str(weapon_1.magazine_size) + '/' + str(weapon_1.magazines_left) +'].'
            else:
                health_status = 'Soldier: 0 HP. '
                agent_status = 'Soldier Status: Dead. Soldier Weapon: N/A. '

            if self.target is not None and self.target.ready:
                health_status = health_status + 'Target: ' + str(self.target.health) + ' HP.'
                agent_status = agent_status + 'Target Status: ' + self.target.movement_mode + '.'
            else:
                health_status = health_status + 'Target: 0 HP.'
                agent_status = agent_status + 'Target Status: Dead.'

            egi.text_at_pos(0, 20, health_status)
            egi.text_at_pos(0, 0, agent_status)

    def change_weapons(self, soldier):
        if len(soldier.weapons) > 0:
            for weapon in soldier.weapons:
                weapon.owner = None

        available = self.weapons.copy()
        soldier.weapons = []
        
        while len(soldier.weapons) < 2:
            if len(available) > 1:
                weapon = available[randrange(0, len(available) - 1)]
            else:
                weapon = available[0]

            self.replenish_weapon(weapon)
            soldier.weapons.append(weapon)
            weapon.owner = soldier
            available.remove(weapon)

    def replenish_weapon(self, weapon):
        weapon.rounds_left_in_magazine = 0

        if weapon.name == 'Rifle':
            weapon.magazines_left = 1#6
        elif weapon.name == 'Rocket':
            weapon.magazines_left = 1#4
        elif weapon.name == 'Hand Gun':
            weapon.magazines_left = 1#10
        elif weapon.name == 'Hand Grenade':
            weapon.magazines_left = 1#2
        elif weapon.name == 'Shotgun':
            weapon.magazines_left = 1#5

    def set_agents(self, max_x, max_y):
        if self.shooter == None:
            self.shooter = Agent(world=self, agent_type='shooter')
            self.agents.append(self.shooter)
            self.shooter.path = Path(num_pts=9, looped=True)
            self.shooter.update_hunt_dist()

        if self.target == None:  
            self.target = Agent(world=self, agent_type='target')
            self.agents.append(self.target)

        self.shooter.pos = Vector2D(max_x * 0.2, max_y * 0.2)
        self.shooter.heading = Vector2D(0,1)
        self.shooter.side = self.shooter.heading.perp()
        self.shooter.path.recreate_preset_path(maxx=self.cx, maxy=self.cy)
        
        self.target.pos = Vector2D(max_x /2, max_y / 2)
        self.target.heading = (self.shooter.pos - self.target.pos).get_normalised()
        self.target.side = self.target.heading.perp()
        self.target.current_pt = Vector2D(self.target.pos.x, max_y * 0.25)
        self.target.next_pt = Vector2D(self.target.pos.x, max_y * 0.75)

        self.shooter.ready = True
        self.target.ready = True

    def destroy_agent(self, agent):
        if agent in self.agents:
            self.agents.remove(agent)

        if agent is self.shooter:
            self.shooter = None

            for projectile in self.projectiles:
                projectile.shooter = None
                projectile.left_barrel = True

            for weapon in agent.weapons:
                self.replenish_weapon(weapon)

            agent.weapons = []
        
        if agent is self.target:
            self.target = None

        del agent

    def destroy_projectile(self, projectile):
        print('Destroying projectile. Projectile pool length: ' + str(len(projectile.weapon.projectile_pool)) + '.')

        # remove projectile from world so that it does not render or update
        if projectile in self.projectiles:
            self.projectiles.remove(projectile)

        # return projectile to shooter's projectile pool
        projectile.vel = None
        projectile.target = None
        projectile.explosion_time = None

        projectile.weapon.projectile_pool.append(projectile)
        print('Destroyed projectile. Projectile pool length: ' + str(len(projectile.weapon.projectile_pool)))
    
    def get_hiding_spots(self, hunter, obstacles):
        hiding_spots = []

        # check for possible hiding spots
        for obstacle in obstacles:
            spot = obstacle.hiding_spot
            spot.pos = self.get_hiding_spot_position(hunter, obstacle)
            spot.valid = True
            spot.rank = 0
            total = 0

            # check if spot is in the bounds of the screen
            if spot.pos.x < 0 or spot.pos.x > self.cx or spot.pos.y < 0 or spot.pos.y > self.cy:
                spot.valid = False
            else:
                # check if spot is inside the bounds of an object
                for o in obstacles:
                    if spot.distance(o.pos) < o.radius:
                        spot.valid = False

            # update spot data

            spot.avg_dist_to_hunter = spot.distance(hunter.pos)
            hiding_spots.append(spot)

        return hiding_spots

    def get_hiding_spot_position(self, hunter, obstacle):
        # set the distance between the obstacle and the hiding point
        dist_from_boundary = self.agent_avoid_radius + 5.0
        dist_away = obstacle.radius + dist_from_boundary

        # get the normal vector from the hunter to the hiding point
        to_obstacle = obstacle.pos - hunter.pos
        to_obstacle.normalise()

        # scale size past the obstacle to the hiding location
        return (to_obstacle * dist_away) + obstacle.pos

    def wrap_around(self, pos):
        ''' Treat world as a toroidal space. Updates parameter object pos '''
        max_x, max_y = self.cx, self.cy
        if pos.x > max_x:
            pos.x = pos.x - max_x
        elif pos.x < 0:
            pos.x = max_x - pos.x
        if pos.y > max_y:
            pos.y = pos.y - max_y
        elif pos.y < 0:
            pos.y = max_y - pos.y

    def transform_point(self, point, pos, forward, side):
        ''' Transform the given single point, using the provided position,
            and direction (forward and side unit vectors), to object world space. '''
        # make a copy of the original point (so we don't trash it)
        wld_pt = point.copy()
        # create a transformation matrix to perform the operations
        mat = Matrix33()
        #rotate
        mat.rotate_by_vectors_update(forward, side)
        #and translate
        mat.translate_update(pos.x, pos.y)
        # now transform the point (in place)
        mat.transform_vector2d(wld_pt)
        #done
        return wld_pt

    def transform_points(self, points, pos, forward, side, scale):
        ''' Transform the given list of points, using the provided position,
            direction and scale, to object world space. '''
        # make a copy of original points (so we don't trash them)
        wld_pts = [pt.copy() for pt in points]
        # create a transformation matrix to perform the operations
        mat = Matrix33()
        # scale,
        mat.scale_update(scale.x, scale.y)
        # rotate
        mat.rotate_by_vectors_update(forward, side)
        # and translate
        mat.translate_update(pos.x, pos.y)
        # now transform all the points (vertices)
        mat.transform_vector2d_list(wld_pts)
        # done
        return wld_pts
