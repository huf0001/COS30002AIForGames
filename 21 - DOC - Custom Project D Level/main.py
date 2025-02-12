'''  BoxWorldWindow to test/demo graph (path) search.

Created for COS30002 AI for Games, Lab,
by Clinton Woodward <cwoodward@swin.edu.au>

For class use only. Do not publically share or post this code without
permission.

See readme.txt for details.

'''

from graphics import egi
import pyglet
from pyglet import window, clock
from pyglet.window import key
from pyglet.gl import *
from pyglet.text import Label

from box_world import BoxWorld, search_modes, cfg, soldier_designation

class BoxWorldWindow(pyglet.window.Window):

    # Mouse mode indicates what the mouse "click" should do...
    mouse_modes = {
        key._1: 'clear',
        key._2: 'mud',
        key._3: 'water',
        key._4: 'wall',
        key._5: 'base'
    }
    mouse_mode = 'wall'

    waypoint_indexes = {
        key._0: 0,
        key._1: 1,
        key._2: 2,
        key._3: 3,
        key._4: 4,
        key._5: 5,
        key._6: 6,
        key._7: 7,
        key._8: 8,
        key._9: 9
    }

    # search mode cycles through the search algorithm used by box_world
    search_mode = 0

    def __init__(self, filename, **kwargs):
        kwargs.update({
            'width': 700,
            'height': 700,
            'vsync':True,
            'resizable':True,
        })
        super(BoxWorldWindow, self).__init__(**kwargs)

        # create a pyglet window and set glOptions
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        # needed so that graphs.egi knows where to draw
        egi.InitWithPyglet(self)
        egi.text_color(name='BLACK')

        glClearColor(0.9, 0.9, 0.9, 1.0) # Grey

        #create a world for graph searching
        #filename = kwargs['filename'] #"boxworld2.txt"
        #filename = 'map2.txt'
        self.search_mode = 3
        self.world = BoxWorld.FromFile(filename, self.get_size())
        self.world.window = self
        self.world.reset_navgraph()

        # prep the fps display and some labels
        self.fps_display = clock.ClockDisplay() # None 
        clBlack = (0,0,0, 255)
        self.labels = {
            'mouse':    Label('', x=5, y=self.height-20, color=clBlack),
            'search':   Label('', x=105, y=self.height-20, color=clBlack),
            'status':   Label('', x=225, y=self.height-20, color=clBlack),
            'menu':     Label('', x=400, y=self.height-20, color=clBlack)
        }
        self._update_label()

        # add the extra event handlers we need
        self.add_handers()

        # search limit
        self.limit = 0 # unlimited.

        self.world.walls = self.world.find_walls(self.world.boxes)
        self.world.bases = self.world.find_bases(self.world.boxes)
        self.world.set_waypoints()
        self.world.set_soldiers()

    def _update_label(self, key=None, text='---'):
        if key == 'mouse' or key is None:
            self.labels['mouse'].text = 'Kind: '+ self.mouse_mode
        if key == 'search' or key is None:
            self.labels['search'].text = 'Search: '+ search_modes[self.search_mode]
        if key == 'status' or key is None:
            self.labels['status'].text = 'Status: '+ text
        if key == 'menu' or key is None:
            self.labels['menu'].text = "Menu: " + self.world.current_menu

            if self.world.current_menu == "Editing Waypoints":
                self.labels['menu'].text += " (" + str(self.world.active_waypoint) + ")"

    def add_handers(self):

        @self.event
        def on_resize(cx, cy):
            self.world.resize(cx, cy-25)
            # reposition the labels.
            for key, label in list(self.labels.items()):
                label.y = cy-20

        @self.event
        def on_mouse_press(x, y, button, modifiers):
            if button == 1: # left
                box = self.world.get_box_by_pos(x,y)

                if box:
                    if self.world.current_menu == "Editing Boxes":
                        for agent in self.world.agents:
                            if box == self.world.get_box_by_pos(int(agent.pos.x), int(agent.pos.y)):
                                print("Illegal change. That box has agents in it. Try again when they've all moved to new boxes.")
                                return

                        start_kind = box.kind
                        box.set_kind(self.mouse_mode)

                        if box.kind != start_kind:
                            print("box change")
                            if start_kind == "X":
                                if box in self.world.walls:
                                    self.world.walls.remove(box)
                            elif start_kind == "B":
                                if box in self.world.bases:
                                    if len(self.world.bases) > 0:
                                        self.world.bases.remove(box)
                                    else:
                                        print("Soldiers need to have at least one base.")

                            if box.kind == "X":
                                self.world.walls.append(box)
                            elif box.kind == "B":
                                if len(self.world.bases) >= len(soldier_designation):
                                    print("The maximum no. of soldier bases has already been reached.")
                                else:
                                    self.world.bases.append(box)

                            self.world.reset_navgraph()
                            self.plan_path()
                            self._update_label('status','graph changed')
                    elif self.world.current_menu == "Managing Agents":
                        self.world.manage_agent_in_box(box)
                    elif self.world.current_menu == "Editing Waypoints":
                        self.world.edit_waypoint_node(box)
                        self.world.reset_navgraph()
                        self.plan_path()

        @self.event
        def on_key_press(symbol, modifiers):
            if self.world.current_menu == "Editing Boxes" and symbol in self.mouse_modes:
                self.mouse_mode = self.mouse_modes[symbol]
                self._update_label('mouse')
            elif self.world.current_menu == "Editing Waypoints" and symbol in self.waypoint_indexes:
                self.world.active_waypoint = self.waypoint_indexes[symbol]
                self._update_label('menu')
            elif symbol == key.M:
                self.world.increment_menu()
                self._update_label('menu')
            elif symbol == key.E:
                cfg['EDGES_ON'] = not cfg['EDGES_ON']
            elif symbol == key.L:
                cfg['LABELS_ON'] = not cfg['LABELS_ON']
            elif symbol == key.C:
                cfg['CENTER_ON'] = not cfg['CENTER_ON']
            elif symbol == key.B:
                cfg['BOXLINES_ON'] = not cfg['BOXLINES_ON']
            elif symbol == key.O:
                cfg['PATH_ON'] = not cfg['PATH_ON']
            elif symbol == key.P:
                self.world.paused = not self.world.paused
            elif symbol == key.F:
                self.world.show_fugitive_awareness_range = not self.world.show_fugitive_awareness_range
            elif symbol == key.S:
                self.world.show_soldier_awareness_range = not self.world.show_soldier_awareness_range
            elif symbol == key.W:
                self.world.show_weapon_range = not self.world.show_weapon_range

    def plan_path(self):
        self.world.plan_path(search_modes[self.search_mode], self.limit)
        self._update_label('status', 'path planned')

    def on_draw(self):
        self.clear()
        self.draw()

    def draw(self):
        self.world.draw()

        if self.fps_display:
            self.fps_display.draw()
        for key, label in list(self.labels.items()):
            label.draw()


#==============================================================================

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "map3.txt"
    window = BoxWorldWindow(filename)

    while not window.has_exit:
        window.dispatch_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # show nice FPS bottom right (default)
        delta = clock.tick()
        window.world.update(delta)
        window.draw()
        window.flip()
