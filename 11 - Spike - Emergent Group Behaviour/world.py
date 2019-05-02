'''A 2d world that supports agents with steering behaviour

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from matrix33 import Matrix33
from graphics import egi


class World(object):
    def __init__(self, cx, cy):
        # world size
        self.cx = cx
        self.cy = cy

        # agent fields / collections
        self.predator = None
        self.prey = []
        self.agents = []
        self.obstacles = []

        # variable for whether to show various pieces of agent information on-screen       
        self.show_values = True
        self.show_avoid = False
        self.show_forces = False
        self.show_wander = False

        # other variables for managing input in main
        self.input_menu_open = False
        self.agent_info = False
        self.new_agents = False

        # pause variable
        self.paused = True

    def update(self, delta):
        if not self.paused:
            # if len(self.hunters) > 0 and len(self.evaders) > 0:
            #     self.hiding_spots = self.get_hiding_spots(self.hunters, self.obstacles)

            for agent in self.agents:
                agent.update(delta)

    def render(self):
        for agent in self.agents:
            agent.render()

        for obstacle in self.obstacles:
            obstacle.render()

        if self.show_values:
            modes = 'Agent mode(s): ' + ', '.join(set(agent.mode for agent in self.agents))
            egi.white_pen()
            egi.text_at_pos(0, 0, modes)

    def add_prey(self, prey):
        self.agents.append(prey)
        self.prey.append(prey)

    def destroy(self, agent):
        if agent in self.agents:
            self.agents.remove(agent)

        if agent in self.evaders:
            self.evaders.remove(agent)
        
        if agent in self.hunters:
            self.hunters.remove(agent)

        del agent

    # def get_hiding_spots(self, hunters, obstacles):
    #     hiding_spots = []

    #     # check for possible hiding spots
    #     for hunter in hunters:
    #         for obstacle in obstacles:
    #             spot = obstacle.hiding_spot
    #             spot.pos = self.get_hiding_spot_position(hunter, obstacle)
    #             spot.valid = True
    #             spot.rank = 0
    #             total = 0

    #             # check if spot is in the bounds of the screen
    #             if spot.pos.x < 0 or spot.pos.x > self.cx or spot.pos.y < 0 or spot.pos.y > self.cy:
    #                 spot.valid = False
    #             else:
    #                 # check if spot is inside the bounds of an object
    #                 for o in obstacles:
    #                     if spot.distance(o.pos) < o.radius:
    #                         spot.valid = False
 
    #             # update spot data                                      
    #             for hunter in hunters:
    #                 total += spot.distance(hunter.pos)

    #             spot.avg_dist_to_hunter = total / len(hunters)
    #             hiding_spots.append(spot)

    #     return hiding_spots

    # def get_hiding_spot_position(self, hunter, obstacle):
    #     # set the distance between the obstacle and the hiding point
    #     dist_from_boundary = self.agent_avoid_radius + 5.0
    #     dist_away = obstacle.radius + dist_from_boundary

    #     # get the normal vector from the hunter to the hiding point
    #     to_obstacle = obstacle.pos - hunter.pos
    #     to_obstacle.normalise()

    #     # scale size past the obstacle to the hiding location
    #     return (to_obstacle * dist_away) + obstacle.pos

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
