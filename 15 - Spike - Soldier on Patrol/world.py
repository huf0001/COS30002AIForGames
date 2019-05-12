'''A 2d world that supports agents with steering behaviour

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from matrix33 import Matrix33
from graphics import egi
from agent import Agent


class World(object):
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.wall_margin = 5

        self.agents = []
        self.target = None
        self.shooter = None
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

    def update(self, delta):
        if not self.paused:
            if self.target.sub_mode == 'Evading':
                self.hiding_spots = self.get_hiding_spots(self.shooter, self.obstacles)

            for agent in self.agents:
                agent.update(delta)

            for projectile in self.projectiles:
                projectile.update(delta)

    def render(self):
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
            egi.text_at_pos(0, 0, 'Shooter Weapon: ' + self.shooter.weapon + '. Target Motion: ' + self.target.mode + '.')

    def destroy(self, agent):
        if agent in self.agents:
            self.agents.remove(agent)

        if agent in self.evaders:
            self.evaders.remove(agent)
        
        if agent in self.hunters:
            self.hunters.remove(agent)

        del agent

    def destroy_projectile(self, projectile):
        print('destroying projectile. projectile pool length: ' + str(len(projectile.shooter.projectile_pool)))
        # remove projectile from world so that it does not render or update
        if projectile in self.projectiles:
            self.projectiles.remove(projectile)

        # return projectile to shooter's projectile pool
        projectile.vel = None
        projectile.target = None
        projectile.p_type = None
        projectile.explosion_time = None
        projectile.shooter.projectile_pool.append(projectile)
        print('destroyed projectile. projectile pool length: ' + str(len(projectile.shooter.projectile_pool)))

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

    def set_agents(self, max_x, max_y):
        if self.shooter == None:
            self.shooter = Agent(world=self, agent_type='shooter', weapon='Rifle')
            self.agents.append(self.shooter)
        if self.target == None:  
            self.target = Agent(world=self, agent_type='target', mode='Stationary')
            self.agents.append(self.target)

        self.shooter.pos = Vector2D(max_x * 0.8, max_y / 2)
        self.shooter.heading = (self.target.pos - self.shooter.pos).get_normalised()
        self.shooter.side = self.shooter.heading.perp()
        
        self.target.pos = Vector2D(max_x * 0.2, max_y / 2)
        self.target.heading = (self.shooter.pos - self.target.pos).get_normalised()
        self.target.side = self.target.heading.perp()
        self.target.current_pt = Vector2D(self.target.pos.x, max_y * 0.25)
        self.target.next_pt = Vector2D(self.target.pos.x, max_y * 0.75)

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
