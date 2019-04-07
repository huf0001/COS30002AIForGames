'''Autonomous Agent Movement: Seek, Arrive and Flee

Created for COS30002 AI for Games, Lab,
by Clinton Woodward <cwoodward@swin.edu.au>

For class use only. Do not publically share or post this code without
permission.

Notes:
* The graphics.py module provides a simple wrapper around pyglet to make
  things easier - hence Easy Graphics Interface (egi).

* If you want to respond to a key press, see the on_key_press function.
* The world contains the agents. In the main loop we tell the world
  to update() and then render(), which then tells each of the agents
  it has.

Updated 2019-03-17

'''
from graphics import egi, KEY
from pyglet import window, clock
from pyglet.gl import *

from vector2d import Vector2D
from world import World
from agent import Agent, AGENT_MODES  # Agent with seek, arrive, flee and pursuit

NUM_KEYS = {
	KEY._0: 0,
	KEY._1: 1,
	KEY._2: 2,
	KEY._3: 3,
	KEY._4: 4,
	KEY._5: 5,
	KEY._6: 6,
	KEY._7: 7,
	KEY._8: 8,
	KEY._9: 9
}


def on_mouse_press(x, y, button, modifiers):
    if button == 1:  # left
        world.target = Vector2D(x, y)


def on_key_press(symbol, modifiers):
    if symbol == KEY.P:
        world.paused = not world.paused
    elif symbol == KEY.F:
    	for agent in world.agents:
    		agent.applying_friction = not agent.applying_friction
    elif symbol == KEY.A:
    	world.new_agents = True
    	return
    elif world.new_agents and symbol in NUM_KEYS:
    	loop = NUM_KEYS[symbol]

    	while loop > 0:
    		world.agents.append(Agent(world=world, mode=world.agent_mode))
    		loop -= 1
    elif symbol in AGENT_MODES:
    	mode = AGENT_MODES[symbol]

    	if mode == 'pursuit' and len(world.agents) == 1:
    		return

    	world.agent_mode = mode

    	for agent in world.agents:
    		agent.mode = mode

    	if mode == 'pursuit':
    		world.agents[0].mode = 'flee'
    		world.evader = world.agents[0]

    world.new_agents = False


def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy


if __name__ == '__main__':

    # create a pyglet window and set glOptions
    win = window.Window(width=500, height=500, vsync=True, resizable=True)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # needed so that egi knows where to draw
    egi.InitWithPyglet(win)
    # prep the fps display
    fps_display = clock.ClockDisplay()
    # register key and mouse event handlers
    win.push_handlers(on_key_press)
    win.push_handlers(on_mouse_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(500, 500)
    # add one agent
    world.agents.append(Agent(world=world, mode=world.agent_mode))
    world.evader = world.agents[0]
    # unpause the world ready for movement
    world.paused = False

    while not win.has_exit:
        win.dispatch_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # show nice FPS bottom right (default)
        delta = clock.tick()
        world.update(delta)
        world.render()
        fps_display.draw()
        # swap the double buffer
        win.flip()

