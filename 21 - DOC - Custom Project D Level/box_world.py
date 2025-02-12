''' Basic square grid based world (BoxWorld) to test/demo path planning.

Created for COS30002 AI for Games, Lab,
by Clinton Woodward <cwoodward@swin.edu.au>

For class use only. Do not publically share or post this code without
permission.

See readme.txt for details. Look for ### comment lines.

Note that the box world "boxes" (tiles) are created and assigned an index (idx)
value, starting from the origin in the bottom left corder. This matches the
convention of coordinates used by pyglet which uses OpenGL, rather than a
traditional 2D graphics with the origin in the top left corner.

   +   ...
   ^   5 6 7 8 9
   |   0 1 2 3 4
 (0,0) ---> +

A BoxWorld can be loaded from a text file. The file uses the following format.

* Values are separated by spaces or tabs (not commas)
* Blank lines or lines starting with # (comments) are ignored
* The first data line is two integer values to specify width and height
* The second row specifies the Start and the Target boxes as index values.
    S 10 T 15
* Each BowWorld row is the specified per line of the text file.
    - Each type is specified by a single character ".", "~", "m" or "#".
    - Number of tile values must match the number of columns
* The number of rows must match the number of specified rows.

Example BoxWorld map file.

# This is a comment and is ignored
# First specify the width x height values
6 5
# Second specify the start and target box indexes
0 17
# Now specify each row of column values
. . . . . .
~ ~ X . . .
. ~ X ~ . .
. . X . . .
. m m m . .
# Note the number of rows and column values match

'''

from graphics import egi
import pyglet
from pyglet.gl import *
from point2d import Point2D
from graph import SparseGraph, Node, Edge
from searches import SEARCHES
from math import hypot
from agent import Agent
from random import random, randrange, uniform
from weapon import Weapon
from vector2d import Vector2D
from matrix33 import Matrix33


box_kind = ['.','m','~','X','B']

box_kind_map = {
    'clear': '.',
    'mud':   'm',
    'water': '~',
    'wall':  'X',
    'base':  'B'
}

no_edge = ['X'] # box kinds that don't have edges.

edge_cost_matrix = [
    # '.'   'm'   '~'   'X'   'B'
    [ 1.0,  2.0,  5.0, None,  1.0], # '.'
    [ 2.0,  4.0,  9.0, None,  2.0], # 'm'
    [ 5.0,  9.0, 10.0, None,  5.0], # '~'
    [None, None, None, None, None], # 'X <- NO edges to walls.
    [ 1.0,  2.0,  5.0, None,  1.0], # 'B'
]

min_edge_cost = 1.0 # must be min value for heuristic cost to work

def edge_cost(k1, k2):
    k1 = box_kind.index(k1)
    k2 = box_kind.index(k2)
    return edge_cost_matrix[k1][k2]


box_kind_color = {
    '.': (1.0, 1.0, 1.0, 1.0), # clear, White
    'm': (0.6, 0.6, 0.5, 1.0), # mud,   Brown-ish
    '~': (0.5, 0.5, 1.0, 1.0), # water, Light blue
    'X': (0.2, 0.2, 0.2, 1.0), # walls, Dark grey
    'B': (0.627, 0.627, 0.627, 1) # bases, Light grey
}


cfg = {
    'LABELS_ON': False,
    'EDGES_ON': False,
    'CENTER_ON': False,
    'BOXLINES_ON': False,
    'BOXUSED_ON': False,
    'TREE_ON': False,
    'PATH_ON': False,
}

soldier_designation = {
    1: "A",
    2: "B",
    3: "C",
    4: "D",
    5: "E",
    6: "F",
    7: "G",
    8: "H",
    9: "I"
}

search_modes = list(SEARCHES.keys())



class Waypoint(object):
    def __init__(self, index=0):
        self.nodes = []
        self.index = index

class Box(object):
    '''A single box for boxworld. '''

    def __init__(self, coords=(0,0,0,0), kind='.'):
        # keep status
        self.kind = kind
        self.color = box_kind_color[kind]
        self.marker = None
        # nav graph node
        self.node = None
        self.idx = -1
        # pretty labels...
        self.idx_label = None
        self.pos_label = None
        self.waypoint_label = None
        self.marker_label = None
        # position using coordinates
        self.reposition(coords)
        self.last_accessor = ""
        self.waypoint = None

    def get_vc(self, accessor):
        self.last_accessor = accessor
        return self._vc

    def reposition(self, coords):
        # top, right, bottom, left
        pts = self.coords = coords
        # points for drawing
        self._pts = (
            Point2D(pts[3], pts[0]), # top left
            Point2D(pts[1], pts[0]), # top right
            Point2D(pts[1], pts[2]), # bottom right
            Point2D(pts[3], pts[2])  # bottom left
        )
        # vector-centre point
        self._vc = Vector2D((pts[1]+pts[3])/2.0, (pts[0]+pts[2])/2.0)
        self.position = Vector2D((pts[1]+pts[3])/2.0, (pts[0]+pts[2])/2.0)
        self.radius = ((self._pts[1].x - self._pts[3].x) + (self._pts[1].y - self._pts[3].y))/5
        # labels may need to be updated
        self._reposition_labels()

    def _reposition_labels(self):
        # reposition labels if we have any
        if self.idx_label:
            self.idx_label.x = self._vc.x
            self.idx_label.y = self._vc.y
            self.pos_label.x = self._vc.x
            self.pos_label.y = self._vc.y

        if self.marker_label:
            self.marker_label.x = self._vc.x
            self.marker_label.y = self._vc.y
            #self._vc.y - (self.marker_label.content_height // 2)

        if self.waypoint_label:
            self.waypoint_label.x = self._vc.x
            self.waypoint_label.y = self._vc.y

    def set_kind(self, kind):
        'Set the box kind (type) using string a value ("water","mud" etc)'
        kind = box_kind_map.get(kind, kind)
        try:
            self.kind = kind
            self.color = box_kind_color[kind]
        except KeyError:
            print('not a known tile kind "%s"' % kind)

    def draw(self):
        # draw filled box
        if self.kind is not "X":
            egi.set_pen_color(self.color)
            egi.closed_shape(self._pts, filled=True)
        else:
            egi.white_pen()
            egi.closed_shape(self._pts, filled=True)

            egi.set_pen_color(self.color)
            egi.set_stroke(5)
            egi.circle(self._vc, self.radius)

            egi.set_stroke(1)

        # draw box border
        if cfg['BOXLINES_ON']:
            egi.set_pen_color((.7,.7,.7,1))
            egi.closed_shape(self._pts, filled=False)
        # centre circle
        if cfg['CENTER_ON']:
            egi.set_pen_color((.3,.3,1,1))
            egi.circle(self._vc, 5)
        # box position (simple column,row) (or x,y actually)
        if self.node:
            if cfg['LABELS_ON']:
                if not self.idx_label:
                    info = "%d" % self.idx
                    self.idx_label = pyglet.text.Label(info, color=(0,0,0,255),
                                                       anchor_x="center",
                                                       anchor_y="top")
                    info = "(%d,%d)" % (self.pos[0], self.pos[1])
                    self.pos_label = pyglet.text.Label(info, color=(0,0,0,255),
                                                       anchor_x="center",
                                                       anchor_y="bottom")
                    self._reposition_labels()
                self.idx_label.draw()
                #self.pos_label.draw()
        if self.marker:
            if not self.marker_label or self.marker_label.text != self.marker:
                self.marker_label = pyglet.text.Label(self.marker,
                                                      color=(255,0,0,255),
                                                      bold=True,
                                                      anchor_x="center",
                                                      anchor_y="center")
                self._reposition_labels()
            self.marker_label.draw()


class BoxWorld(object):
    '''A world made up of boxes. '''

    def __init__(self, nx, ny, cx, cy):
        self.waypoints = []
        self.walls = []
        self.bases = []
        self.soldiers = []
        self.fugitives = []
        self.agents = []
        self.weapons = []
        self.projectiles = []
        self.set_weapons()
        self.boxes = [None]*nx*ny
        self.nx, self.ny = nx, ny # number of box (squares)

        for i in range(len(self.boxes)):
            self.boxes[i] = Box()
            self.boxes[i].idx = i

        # use resize to set all the positions correctly
        self.cx = self.cy = self.wx = self.wy = None
        self.original_cx = cx
        self.original_cy = cy
        self.scale_scalar = 1
        self.scale_vector = Point2D(1, 1)
        self.resize(cx, cy)

        # create nav_graph
        self.path = None
        self.graph = None
        self.diagonal = '_max'
        self.reset_navgraph()
        self.targets = []
        self.paused = False
        self.window = None
        self.cfg = cfg

        self.active_waypoint = 0 # currently being edited
        self.current_waypoint = 0 # the next waypoint in the patrol of the soldier agents
        self.last_waypoint = 9 # the previous waypoint in the patrol of the soldier agents

        self.show_soldier_awareness_range = False
        self.show_fugitive_awareness_range = False
        self.show_weapon_range = False

        self.extra_fugitive_count = 0

        self.current_menu = "Managing Agents"
        self.current_menu_index = 0

        i = 0

        while i < 10:
            self.waypoints.append(Waypoint(index=i))
            i += 1

    # Setup Methods----------------------------------------------------------------------------------------------------------------------------------

    def _add_edge(self, from_idx, to_idx, distance=1.0):
        b = self.boxes
        if b[to_idx].kind not in no_edge: # stone wall
            cost = edge_cost(b[from_idx].kind, b[to_idx].kind)
            self.graph.add_edge(Edge(from_idx, to_idx, cost*distance))

    def _hypot(self, idx1, idx2):
        '''Return the straight line distance between two points on a 2-D
        Cartesian plane. Argh, Pythagoras... trouble maker. '''
        x1, y1 = self.boxes[idx1].pos
        x2, y2 = self.boxes[idx2].pos
        return hypot(x1-x2, y1-y2) * min_edge_cost

    def _manhattan(self, idx1, idx2):
        ''' Manhattan distance between two nodes in boxworld, assuming the
        minimal edge cost so that we don't overestimate the cost). '''
        x1, y1 = self.boxes[idx1].pos
        x2, y2 = self.boxes[idx2].pos
        return (abs(x1-x2) + abs(y1-y2)) * min_edge_cost

    def _max(self, idx1, idx2):
        '''Return the straight line distance between two points on a 2-D
        Cartesian plane. Argh, Pythagoras... trouble maker. '''
        x1, y1 = self.boxes[idx1].pos
        x2, y2 = self.boxes[idx2].pos
        return max(abs(x1-x2),abs(y1-y2)) * min_edge_cost

    def find_bases(self, boxes):
        bases = []

        for box in boxes:
            if box.kind == "B":
                bases.append(box)

        return bases

    def find_walls(self, boxes):
        walls = []

        for box in boxes:
            if box.kind == "X":
                walls.append(box)

        return walls    

    def set_soldiers(self):
        available = []

        i = 1

        while i <= len(soldier_designation):
            available.append(soldier_designation[i])
            i += 1

        if len(self.soldiers) > 0:
            for soldier in self.soldiers:
                name_array = soldier.name.split()

                if name_array[len(name_array) - 1] in available:
                    available.remove(name_array[len(name_array) - 1])

        print(available)
        occupied = []

        for soldier in self.soldiers:
            occupied.append(soldier.box)

        i = 0

        for base in self.bases:
            if base not in occupied and len(self.soldiers) < len(self.bases):
                self.soldiers.append(Agent(world=self, agent_type="soldier", box=base.idx, name="Soldier " + available[i]))
                i += 1


        for soldier in self.soldiers:
            self.agents.append(soldier)

        self.soldiers[0].target = self.waypoints[0].nodes[0]
        self.soldiers[0].plan_path(search_modes[self.window.search_mode], self.window.limit)

    def set_waypoints(self):
        i = 0
        j = 0

        while i < len(self.waypoints):

            while j < len(self.waypoints[i].nodes):
                self.waypoints[i].nodes[j] = self.boxes[self.waypoints[i].nodes[j]]
                self.waypoints[i].nodes[j].waypoint = i
                j += 1

            j = 0                
            i += 1  
                  
    def set_weapons(self):
        self.weapons.append(Weapon(
            world = self, 
            name = 'Rifle', 
            cooldown = 1.5,                 # max rpm of 0.5 sec / round
            effective_range = 2300,    # effective range 2300 m
            speed = 200,
            damage = 50, 
            damage_factor = 1,
            reload_time = 2.6, 
            magazine_size = 4, 
            magazines = 1000000000,#6, 
            accuracy_modifier = 0,
            stamina_drain=4))
        self.weapons.append(Weapon(
            world = self, 
            name = 'Hand Gun', 
            cooldown = 0.286,               # max rpm
            effective_range = 122.7,    # effective range 122.7 m
            speed = 200,
            damage = 20, 
            damage_factor = 1,
            reload_time = 1.8, 
            magazine_size = 12, 
            magazines = 1000000000,#10, 
            accuracy_modifier = 5,
            stamina_drain=2))
        self.weapons.append(Weapon(
            world = self, 
            name = 'Shotgun', 
            cooldown = 1,                   # max rpm of 1 sec / round
            effective_range = 6 * 5,       # estimated effective range 5 m 
            speed = 200,
            damage = 20,                    # multiple pellets; damage is spread out amongst them
            damage_factor = 3,
            reload_time = 6, 
            magazine_size = 12, 
            magazines = 1000000000,#5, 
            accuracy_modifier = 5,
            stamina_drain=3))

    # Recurring Methods------------------------------------------------------------------------------------------------------------------------------

    def update(self, delta):
        if not self.paused:
            if len(self.soldiers) == 0:
                self.respawn_all_soldiers()

            for agent in self.agents:
                agent.update(delta)

            for box in self.boxes:
                if box._vc != box.position:
                    # print("Box " + str(box.idx) + " has been moved by " + box.last_accessor)
                    box._vc = box.position.copy()

            for projectile in self.projectiles:
                projectile.update(delta)

    def draw(self):
        for box in self.boxes:
            box.draw()

        if self.current_menu == "Editing Waypoints":
            for waypoint in self.waypoints:
                if waypoint.index == self.active_waypoint:
                    egi.green_pen()
                else:
                    egi.yellow_pen()

                for node in waypoint.nodes:
                    egi.closed_shape(node._pts, filled=True)

                    if not node.waypoint_label:
                        info = "%d" % waypoint.index
                        node.waypoint_label = pyglet.text.Label(info, color=(0,0,0,255),
                                                           anchor_x="center",
                                                           anchor_y="top")
                    node._reposition_labels()
                    node.waypoint_label.draw()

        if cfg['EDGES_ON']:
            egi.set_pen_color(name='LIGHT_BLUE')
            for node, edges in self.graph.edgelist.items():
                # print node, edges
                for dest in edges:
                    egi.line_by_pos(self.boxes[node]._vc, self.boxes[dest]._vc)

        if self.path:
            # put a circle in the visited boxes?
            if cfg['BOXUSED_ON']:
                egi.set_pen_color(name="GREEN")
                for i in self.path.closed:
                    egi.circle(self.boxes[i]._vc, 10)

            if cfg['TREE_ON']:
                egi.set_stroke(3)
                # Show open edges
                route = self.path.route
                egi.set_pen_color(name='GREEN')
                for i in self.path.open:
                    egi.circle(self.boxes[i]._vc, 10)
                # show the partial paths considered
                egi.set_pen_color(name='ORANGE')
                for i,j in route.items():
                    egi.line_by_pos(self.boxes[i]._vc, self.boxes[j]._vc)
                egi.set_stroke(1)

            if cfg['PATH_ON']:
                # show the final path delivered
                egi.set_pen_color(name='RED')
                egi.set_stroke(2)
                path = self.path.path
                for i in range(1,len(path)):
                    egi.line_by_pos(self.boxes[path[i-1]]._vc, self.boxes[path[i]]._vc)
                egi.set_stroke(1)

        for agent in self.agents:
            agent.render()

        for projectile in self.projectiles:
            projectile.render()

    # Triggered Methods------------------------------------------------------------------------------------------------------------------------------

    def increment_menu(self):
        self.current_menu_index += 1

        if self.current_menu_index >= 3:
            self.current_menu_index = 0

        if self.current_menu_index == 0:
            self.current_menu = "Managing Agents"
        elif self.current_menu_index == 1:
            self.current_menu = "Editing Boxes"
        elif self.current_menu_index == 2:
            self.current_menu = "Editing Waypoints"
        else:
            print("BoxWorld.increment_menu(): invalid menu index")

    def plan_path(self, search, limit):
        '''Conduct a nav-graph search from the current world start node to the
        current target node, using a search method that matches the string
        specified in `search`.
        '''

        for agent in self.agents:
            agent.plan_path(search, limit)

    def reset_navgraph(self):
        ''' Create and store a new nav graph for this box world configuration.
        The graph is build by adding NavNode to the graph for each of the
        boxes in box world. Then edges are created (4-sided).
        '''
        self.path = None # invalid so remove if present
        self.graph = SparseGraph()
        # Set a heuristic cost function for the search to use
        
        if self.diagonal == '_manhattan':
            self.graph.cost_h = self._manhattan
        elif self.diagonal == '_hypot':
            self.graph.cost_h = self._hypot
        elif self.diagonal == '_max':
            self.graph.cost_h = self._max

        nx, ny = self.nx, self.ny
        # add all the nodes required
        for i, box in enumerate(self.boxes):
            box.pos = (i % nx, i // nx) #tuple position
            box.node = self.graph.add_node(Node(idx=i))
        # build all the edges required for this world
        for i, box in enumerate(self.boxes):
            # four sided N-S-E-W connections
            if box.kind in no_edge:
                continue
            # UP (i + nx)
            if (i+nx) < len(self.boxes):
                self._add_edge(i, i+nx)
            # DOWN (i - nx)
            if (i-nx) >= 0:
                self._add_edge(i, i-nx)
            # RIGHT (i + 1)
            if (i%nx + 1) < nx:
                self._add_edge(i, i+1)
            # LEFT (i - 1)
            if (i%nx - 1) >= 0:
                self._add_edge(i, i-1)
            # Diagonal connections
            # UP LEFT(i + nx - 1)
            j = i + nx
            if (j-1) < len(self.boxes) and (j%nx - 1) >= 0:
               self._add_edge(i, j-1, 1.4142) # sqrt(1+1)
            # UP RIGHT (i + nx + 1)
            j = i + nx
            if (j+1) < len(self.boxes) and (j%nx + 1) < nx:
               self._add_edge(i, j+1, 1.4142)
            # DOWN LEFT(i - nx - 1)
            j = i - nx
            if (j-1) >= 0 and (j%nx - 1) >= 0:
               # print (i, j, j%nx)
               self._add_edge(i, j-1, 1.4142)
            # DOWN RIGHT (i - nx + 1)
            j = i - nx
            if (j+1) >= 0 and (j%nx +1) < nx:
                self._add_edge(i, j+1, 1.4142)        

    def resize(self, cx, cy):
        self.cx, self.cy = cx, cy # world size
        self.wx = (cx-1) // self.nx
        self.wy = (cy-1) // self.ny # int div - box width/height

        self.scale_vector = Point2D(cx / self.original_cx, cy / self.original_cy)
        self.scale_scalar = (self.scale_vector.x + self.scale_vector.y) / 2

        for i in range(len(self.boxes)):
            # basic positions (bottom left to top right)
            x = (i % self.nx) * self.wx
            y = (i // self.nx) * self.wy
            # top, right, bottom, left
            coords = (y + self.wy -1, x + self.wx -1, y, x)
            self.boxes[i].reposition(coords)

        for agent in self.agents:
            agent.radius = agent.radius_standard * self.scale_scalar
            # agent.avoid_radius = agent.avoid_radius_standard * self.scale_scalar
            agent.awareness_radius = agent.awareness_radius_standard * self.scale_scalar
            agent.pos = agent.box._vc

            if agent.current_node_box != None:
                agent.current_node_pos = agent.current_node_box._vc

        for weapon in self.weapons:
            weapon.effective_range = weapon.effective_range_standard * self.scale_scalar

    # Utility Methods: Get Box By METHOD-------------------------------------------------------------------------------------------------------------

    def get_box_by_index(self, ix, iy):
        idx = (self.nx * iy) + ix
        return self.boxes[idx] if idx < len(self.boxes) else None

    def get_box_by_pos(self, x, y):
        idx = (self.nx * (y // self.wy)) + (x // self.wx)
        return self.boxes[idx] if idx < len(self.boxes) else None

    # Utility Methods: Waypoints---------------------------------------------------------------------------------------------------------------------

    def get_current_waypoint_node(self):
        return self.waypoints[self.current_waypoint].nodes[0]

    def edit_waypoint_node(self, node):
        active_waypoint = self.waypoints[self.active_waypoint]

        if node.waypoint == None:
            active_waypoint.nodes.append(node)
            node.waypoint = active_waypoint.index
        elif node.waypoint == self.active_waypoint:
            active_waypoint.nodes.remove(node)
            node.waypoint = None
        else:
            print("Cannot have a box as part of two waypoints.")

    def update_waypoint(self, trigger):
        if trigger == self.current_waypoint:
            self.last_waypoint = self.current_waypoint
            self.current_waypoint += 1

            if self.current_waypoint >= len(self.waypoints):
                self.current_waypoint = 0

            while len(self.waypoints[self.current_waypoint].nodes) == 0:
                self.current_waypoint += 1

                if self.current_waypoint >= len(self.waypoints):
                    self.current_waypoint = 0
            print("Progressed Waypoint. Last Waypoint: " + str(self.last_waypoint) + ", Current Waypoint: " + str(self.current_waypoint) + ".")
        elif trigger == self.last_waypoint:
            self.current_waypoint = self.last_waypoint
            self.last_waypoint -= 1

            if self.last_waypoint < 0:
                self.last_waypoint = len(self.waypoints) - 1

            while len(self.waypoints[self.last_waypoint].nodes) == 0:
                self.last_waypoint -= 1

                if self.last_waypoint < 0:
                    self.last_waypoint = len(self.waypoints) - 1

            print("Regressed Waypoint. Last Waypoint: " + str(self.last_waypoint) + ", Current Waypoint: " + str(self.current_waypoint) + ".")
        else:
            print("Error: Invalid waypoint triggered. Last Waypoint: " + str(self.last_waypoint) + ", Current Waypoint: " + str(self.current_waypoint) + ", Triggered Waypoint: " + str(trigger) + ".")

    # Utility Methods: Agent Creation and Destruction------------------------------------------------------------------------------------------------

    def destroy_agent(self, deceased):
        if deceased.agent_type == "soldier":
            self.destroy_soldier(deceased)
        elif deceased.agent_type == "fugitive":
            self.destroy_fugitive(deceased)
        else:
            print("Invalid agent type to destroy.")

    def destroy_fugitive(self, deceased):
        if deceased in self.fugitives:
            self.fugitives.remove(deceased)

        self.agents.remove(deceased)
        print(deceased.name + " destroyed")
        del deceased

    def destroy_soldier(self, deceased):
        # set waypoints to suit the new soldier leader
        if deceased == self.soldiers[0] and len(self.soldiers) > 1:
            path = deceased.calculate_path(search_modes[self.window.search_mode], self.soldiers[1].box, self.window.limit)
            last_node = None

            for node in path.path:
                node = self.boxes[node]

                if node.waypoint is not None and last_node is not None and last_node.waypoint is not node.waypoint:
                    self.update_waypoint(node.waypoint)

                last_node = node

        # destroy the soldier
        if deceased in self.soldiers:
            self.soldiers.remove(deceased)

        self.agents.remove(deceased)
        print(deceased.name + " destroyed")    
        del deceased

    def respawn_all_soldiers(self):
        self.current_waypoint = 0
        self.last_waypoint = len(self.waypoints) - 1

        i = 0

        while i > 0 and len(self.waypoints[self.last_waypoint].nodes) == 0:
            self.last_waypoint -= 1

        if self.last_waypoint == 0:
            print("You need more than one waypoint for soldiers to be able to patrol")
            return

        self.set_soldiers()

    def manage_agent_in_box(self, box):
        destroyed = False

        # destroy agents in box
        for agent in self.agents:
            if agent.box == box:
                self.destroy_agent(agent)
                destroyed = True

        if destroyed:
            return

        # check if box is a wall
        if box.kind == "X":
            print("Can't spawn a new fugitive inside a wall.")
            return

        # spawn new fugitive in box
        self.extra_fugitive_count += 1
        fugitive = Agent(world=self, box=box.idx, name="Fugitive Token " + str(self.extra_fugitive_count))
        self.fugitives.append(fugitive)
        self.agents.append(fugitive)

    # Utility Methods: Weapons-----------------------------------------------------------------------------------------------------------------------

    def change_weapons(self, soldier):
        print("changing weapons for " + soldier.agent_type)
        if len(soldier.weapons) > 0:
            for weapon in soldier.weapons:
                weapon.owner = None

        available = self.weapons.copy()

        soldier.weapons = []
        
        while len(soldier.weapons) < 1 and len(available) > 0:
            if len(available) > 1:
                weapon = available[randrange(0, len(available))]
            else:
                weapon = available[0]

            self.replenish_weapon(weapon)
            soldier.weapons.append(weapon.copy())
            weapon.owner = soldier
            available.remove(weapon)
            print("added weapon " + weapon.name)
    
    def destroy_projectile(self, projectile):
        # remove projectile from world so that it does not render or update
        if projectile in self.projectiles:
            self.projectiles.remove(projectile)

        # return projectile to shooter's projectile pool
        projectile.vel = None
        projectile.target = None
        projectile.explosion_time = None
        projectile.owner_on_firing = None
        projectile.weapon.projectile_pool.append(projectile)
    
    def replenish_weapon(self, weapon):
        weapon.rounds_left_in_magazine = 0

        if weapon.name == 'Rifle':
            weapon.magazines_left = 1000000000#6
        elif weapon.name == 'Rocket':
            weapon.magazines_left = 1000000000#4
        elif weapon.name == 'Hand Gun':
            weapon.magazines_left = 1000000000#10
        elif weapon.name == 'Hand Grenade':
            weapon.magazines_left = 1000000000#2
        elif weapon.name == 'Shotgun':
            weapon.magazines_left = 1000000000#5

    # Utility Methods: Transforming Points-----------------------------------------------------------------------------------------------------------

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

    # Read From File Setup Method--------------------------------------------------------------------------------------------------------------------

    @classmethod
    def FromFile(cls, filename, pixels=(500,500) ):
        '''Support a the construction of a BoxWorld map from a simple text file.
        See the module doc details at the top of this file for format details.
        '''
        # open and read the file
        f = open(filename)
        lines = []
        for line in f.readlines():
            line = line.strip()
            if line and not line.startswith('#'):
                lines.append(line)
        f.close()
        # first line is the number of boxes width, height
        nx, ny = [int(bit) for bit in lines.pop(0).split()]
        # Create a new BoxWorld to store all the new boxes in...
        cx, cy = pixels
        world = BoxWorld(nx, ny, cx, cy)

        # Get and set the waypoints
        world.current_waypoint, world.last_waypoint = [int(bit) for bit in lines.pop(0).split()]

        n = int(lines.pop(0))
        i = 0
        while i < n:
            nodes = lines.pop(0).split()
            for node in nodes:
                world.waypoints[i].nodes.append(int(node))
            i += 1

        # Ready to process each line
        assert len(lines) == ny, "Number of rows doesn't match data. Rows: " + str(len(lines)) + ", Rows in data: " + str(ny)
        # read each line
        idx = 0
        for line in reversed(lines): # in reverse order
            bits = line.split()
            assert len(bits) == nx, "Number of columns doesn't match data."
            for bit in bits:
                bit = bit.strip()
                assert bit in box_kind, "Not a known box type: "+bit
                world.boxes[idx].set_kind(bit)
                idx += 1

        return world