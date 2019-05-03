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
        self.selected_index = 0
        self.selected_variable = 'speed'
        self.change_values = False
        self.value_step = 1

        # pause variable
        self.paused = True

    def update(self, delta):
        if not self.paused:
            for agent in self.agents:
                agent.update(delta)

    def render(self):
        for agent in self.agents:
            agent.render()

        for obstacle in self.obstacles:
            obstacle.render()

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
            egi.text_at_pos(10, self.cy - (5 + 45), 'Alignment Multiplier: ' + str(self.prey[0].alignment_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 60), 'Cohesion Multiplier: ' + str(self.prey[0].cohesion_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 75), 'Fleeing Multiplier: ' + str(self.prey[0].fleeing_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 90), 'Separation Multiplier: ' + str(self.prey[0].separation_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 105), 'Wander Multiplier: ' + str(self.prey[0].wander_multiplier))
            egi.text_at_pos(10, self.cy - (5 + 135), 'Value Step: ' + str(self.value_step))

    def add_prey(self, prey):
        self.agents.append(prey)
        self.prey.append(prey)

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
        elif value == 'fleeing':
            for agent in self.prey:
                agent.fleeing_multiplier += step * sign

                if agent.fleeing_multiplier < 0:
                    agent.fleeing_multiplier = 0
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
            return 'None. P: (Un)Pause Simulation. A: Prey Agent Menu. I: Info Display Menu. O: Obstacles Menu. V: Value Editing Menu.'
        elif self.new_agents:
            return 'New Prey Agents. [N]: Spawn [N] Prey Agents. Backspace: Exit Menu.'
        elif self.agent_info:
            return 'Toggle Display Info. A: Avoidance Boundary. F: Forces. V: Behaviour Values. W: Wander Circles. Backspace: Exit Menu.'
        elif self.obstacle_input:
            return 'Obstacles. O: Spawn New Obstacle. R: Randomise Obstacle Positions. Backspace: Exit Menu.'
        elif self.change_values:
            return 'Value Editing. Up/Down: Select Up/Down. Left/Right: Decrease/Increase Selected. Minus/Plus: Decrease/Increase Value Step. Backspace: Exit Menu.'

    def select_variable(self, change):
        max_index = 6
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
            self.selected_variable = 'alignment'
        elif self.selected_index == 3:
            self.selected_variable = 'cohesion'
        elif self.selected_index == 4:
            self.selected_variable = 'fleeing'
        elif self.selected_index == 5:
            self.selected_variable = 'separation'
        elif self.selected_index == 6:
            self.selected_variable = 'wander'

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
