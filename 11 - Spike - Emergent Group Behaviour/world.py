'''A 2d world that supports agents with steering behaviour

Created for COS30002 AI for Games, by Clinton Woodward <cwoodward@swin.edu.au>
For class use only. Do not publically share or post this code without permission.

'''

from vector2d import Vector2D
from matrix33 import Matrix33
from graphics import egi
from point2d import Point2D


class World(object):
    def __init__(self, cx, cy):
        # world size
        self.cx = cx
        self.cy = cy
        self.wall_margin = 5

        # agent fields / collections
        self.predator = None
        self.prey = []
        self.agents = []
        self.obstacles = []
        self.walls = []
        self.neighbourhood_radius = 60

        # variable for whether to show various pieces of agent information on-screen       
        self.show_values = True
        self.show_avoid = False
        self.show_forces = False
        self.show_wander = False
        self.show_radius = False
        self.show_neighbourhood = False

        # other variables for managing input in main
        self.input_menu_open = False
        self.agent_info = False
        self.new_agents = False
        self.obstacle_input = False
        self.selected_index = 0
        self.selected_variable = 'speed'
        self.change_values = False
        self.value_step = 1

        # pause variable
        self.paused = True

    def update(self, delta):
        if not self.paused:
            self.calculate_neighbours()

            for agent in self.agents:
                agent.update(delta)

    def render(self):
        for wall in self.walls:
            wall.render()

        for obstacle in self.obstacles:
            obstacle.render()

        for agent in self.agents:
            agent.render()

        egi.white_pen()
        egi.text_at_pos(5, 5, 'Current Menu: ' + self.get_menu_text())
        egi.text_at_pos(5, 20, 'Agent mode(s): ' + ', '.join(set(agent.mode for agent in self.agents)))

        if len(self.prey) > 0:
            if self.change_values:
                egi.red_pen()
                y_top = self.cy - (5 + (15 * self.selected_index))
                y_bottom = self.cy - (20 + (15 * self.selected_index))

                pts = [
                        Point2D(5,      y_top),     # top left
                        Point2D(400,    y_top),     # top right
                        Point2D(400,    y_bottom),  # bottom right
                        Point2D(5,      y_bottom)   # bottom left
                    ]
                egi.closed_shape(pts)

            egi.white_pen()
            egi.text_at_pos(10, self.cy - (5 + 15), 'Max Speed: ' + str(self.prey[0].max_speed))
            egi.text_at_pos(10, self.cy - (5 + 30), 'Max Force: ' + str(self.prey[0].max_force))
            egi.text_at_pos(10, self.cy - (5 + 45), 'Neighbourhood Radius: ' + str(self.neighbourhood_radius))
            egi.text_at_pos(10, self.cy - (5 + 60), 'Alignment Multiplier: ' + str(self.prey[0].alignment_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 75), 'Cohesion Multiplier: ' + str(self.prey[0].cohesion_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 90), 'Fleeing Multiplier: ' + str(self.prey[0].fleeing_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 105), 'Fleeing Range: ' + str(self.prey[0].flee_range))
            egi.text_at_pos(10, self.cy - (5 + 120), 'Obstacle Avoidance Multiplier: ' + str(self.prey[0].obstacle_avoidance_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 135), 'Separation Multiplier: ' + str(self.prey[0].separation_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 150), 'Wander Multiplier: ' + str(self.prey[0].wander_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 165), 'Separate by Avoid: ' + str(self.prey[0].separate_by_avoid))
            egi.text_at_pos(10, self.cy - (5 + 195), 'Value Step: ' + str(self.value_step))

    def add_prey(self, prey):
        self.agents.append(prey)
        self.prey.append(prey)

    def calculate_neighbours(self):
        self.predator.neighbours = []
        self.predator.wall_neighbours = []
        self.predator.obstacle_neighbours = []

        for wall in self.walls:
            if self.predator.distance(wall.get_pos(self.predator.pos)) < self.predator.avoid_radius * 3:
                self.predator.wall_neighbours.append(wall)

        for obstacle in self.obstacles:
            if self.predator.distance(obstacle.pos) < self.predator.avoid_radius * 3 + obstacle.radius:
                self.predator.obstacle_neighbours.append(obstacle)

        for prey in self.prey:
            if self.predator.distance(prey.pos) < self.predator.avoid_radius * 3:
                self.predator.neighbours.append(prey)

            prey.agent_neighbours = []
            prey.wall_neighbours = []
            prey.obstacle_neighbours = []

            for agent in self.agents:
                if prey is not agent and prey.distance(agent.pos) < self.neighbourhood_radius:
                    prey.agent_neighbours.append(agent)

            for wall in self.walls:
                if prey.distance(wall.get_pos(prey.pos)) < self.neighbourhood_radius:
                    prey.wall_neighbours.append(wall)

            for obstacle in self.obstacles:
                if prey.distance(obstacle.pos) < self.neighbourhood_radius + obstacle.radius:
                    prey.obstacle_neighbours.append(obstacle)

    def change_value(self, value, step, sign):
        if value == 'speed':
            for agent in self.prey:
                agent.max_speed += step * sign

                if agent.max_speed < 0:
                    agent.max_speed = 0
        elif value == 'force':
            for agent in self.prey:
                agent.max_force += step * sign

                if agent.max_force < 0:
                    agent.max_force = 0
        elif value == 'neighbourhood radius':
            self.neighbourhood_radius += step * sign

            if self.neighbourhood_radius < 0:
                self.neighbourhood_radius = 0
        elif value == 'alignment':
            for agent in self.prey:
                agent.alignment_multiplier += step * sign

                if agent.alignment_multiplier < 0:
                    agent.alignment_multiplier = 0
        elif value == 'cohesion':
            for agent in self.prey:
                agent.cohesion_multiplier += step * sign

                if agent.cohesion_multiplier < 0:
                    agent.cohesion_multiplier = 0
        elif value == 'fleeing multiplier':
            for agent in self.prey:
                agent.fleeing_multiplier += step * sign

                if agent.fleeing_multiplier < 0:
                    agent.fleeing_multiplier = 0
        elif value == 'fleeing range':
            for agent in self.prey:
                agent.flee_range += step * sign

                if agent.flee_range < 0:
                    agent.flee_range = 0
        elif value == 'obstacle avoidance':
            for agent in self.prey:
                agent.obstacle_avoidance_multiplier += step * sign

                if agent.obstacle_avoidance_multiplier < 0:
                    agent.obstacle_avoidance_multiplier = 0
        elif value == 'separation':
            for agent in self.prey:
                agent.separation_multiplier += step * sign

                if agent.separation_multiplier < 0:
                    agent.separation_multiplier = 0
        elif value == 'wander':
            for agent in self.prey:
                agent.wander_multiplier += step * sign

                if agent.wander_multiplier < 0:
                    agent.wander_multiplier = 0
        elif value == 'separate by avoid':
            for agent in self.agents:
                agent.separate_by_avoid = not agent.separate_by_avoid

    def destroy(self, agent):
        if agent in self.agents:
            self.agents.remove(agent)

        if agent in self.evaders:
            self.evaders.remove(agent)
        
        if agent in self.hunters:
            self.hunters.remove(agent)

        del agent

    def get_menu_text(self):
        if not self.input_menu_open:
            return 'None. P: (Un)Pause Simulation. A: Prey Agent Menu. I: Info Display Menu. O: Obstacles Menu. V: Value Editing Menu. 0: Spawn 10 Prey. '
        elif self.new_agents:
            return 'New Prey Agents [ [N]: Spawn [N] Prey Agents (0 Key Spawns 10 Prey Agents) ]. P: (Un)Pause Simulation. Backspace: Exit Menu.'
        elif self.agent_info:
            return 'Toggle Display Info [ A: Avoidance Boundary, F: Forces, V: Behaviour Values, W: Wander Circles ]. 0: Spawn 10 Prey. P: (Un)Pause Simulation. Backspace: Exit Menu.'
        elif self.obstacle_input:
            return 'Obstacles [ N: Spawn New Obstacle, R: Randomise Obstacle Positions ]. 0: Spawn 10 Prey. P: (Un)Pause Simulation. Backspace: Exit Menu.'
        elif self.change_values:
            return 'Value Editing [ Up/Down: Select Up/Down, Left/Right: Decrease/Increase Selected, Minus/Plus: Decrease/Increase Value Step ]. 0: Spawn 10 Prey. P: (Un)Pause Simulation. Backspace: Exit Menu.'

    def select_variable(self, change):
        max_index = 10
        self.selected_index += change

        if self.selected_index < 0:
            self.selected_index = max_index
        elif self.selected_index > max_index:
            self.selected_index = 0

        if self.selected_index == 0:
            self.selected_variable = 'speed'
        elif self.selected_index == 1:
            self.selected_variable = 'force'
        elif self.selected_index == 2:
            self.selected_variable = 'neighbourhood radius'
        elif self.selected_index == 3:
            self.selected_variable = 'alignment'
        elif self.selected_index == 4:
            self.selected_variable = 'cohesion'
        elif self.selected_index == 5:
            self.selected_variable = 'fleeing multiplier'
        elif self.selected_index == 6:
            self.selected_variable = 'fleeing range'
        elif self.selected_index == 7:
            self.selected_variable = 'obstacle avoidance'
        elif self.selected_index == 8:
            self.selected_variable = 'separation'
        elif self.selected_index == 9:
            self.selected_variable = 'wander'
        elif self.selected_index == 10:
            self.selected_variable = 'separate by avoid'

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
