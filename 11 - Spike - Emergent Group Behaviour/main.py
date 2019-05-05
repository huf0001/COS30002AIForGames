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
from agent import Agent, AGENT_MODES
from obstacle import Obstacle
from wall import Wall

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


def on_key_press(symbol, modifiers):
	# exiting menus
	if symbol == KEY.BACKSPACE:
	    world.input_menu_open = False
	    world.new_agents = False
	    world.agent_info = False
	    world.obstacle_input = False
	    world.change_values = False
	# Pause / unpause the game
	elif symbol == KEY.P:
	    world.paused = not world.paused
	# shortcut key for spawning 10 new prey agents
	elif symbol == KEY._0:
	    loop = 10

	    if len(world.prey) > 0:
	        update_values = True
	    else:
	        update_values = False

	    # spawn specified number of agents
	    while loop > 0:
	        prey = Agent(world=world, scale=10, radius=10)

	        if update_values:
	            prey.max_speed = world.prey[0].max_speed
	            prey.max_force = world.prey[0].max_force
	            prey.alignment_multiplier = world.prey[0].alignment_multiplier
	            prey.cohesion_multiplier = world.prey[0].cohesion_multiplier
	            prey.fleeing_multiplier = world.prey[0].fleeing_multiplier
	            prey.obstacle_avoidance_multiplier = world.prey[0].obstacle_avoidance_multiplier
	            prey.separation_multiplier = world.prey[0].separation_multiplier
	            prey.wander_multiplier = world.prey[0].wander_multiplier

	        world.add_prey(prey)
	        loop -= 1
	elif world.input_menu_open:
	    # agent menu
	    if world.new_agents and symbol in NUM_KEYS:
	        loop = NUM_KEYS[symbol]

	        if len(world.prey) > 0:
	            update_values = True
	        else:
	            update_values = False

	        # spawn specified number of agents
	        while loop > 0:
	            prey = Agent(world=world, scale=10, radius=10)

	            if update_values:
	                prey.max_speed = world.prey[0].max_speed
	                prey.max_force = world.prey[0].max_force
	                prey.alignment_multiplier = world.prey[0].alignment_multiplier
	                prey.cohesion_multiplier = world.prey[0].cohesion_multiplier
	                prey.fleeing_multiplier = world.prey[0].fleeing_multiplier
	                prey.obstacle_avoidance_multiplier = world.prey[0].obstacle_avoidance_multiplier
	                prey.separation_multiplier = world.prey[0].separation_multiplier
	                prey.wander_multiplier = world.prey[0].wander_multiplier

	            world.add_prey(prey)
	            loop -= 1
	    # show info menu
	    elif world.agent_info:
	        # agent avoidance
	        if symbol == KEY.A:
	            world.show_avoid = not world.show_avoid
	        # agent vector info
	        elif symbol == KEY.F:
	            world.show_forces = not world.show_forces
	        # neighbourhood info
	        elif symbol == KEY.N:
	            world.show_neighbourhood = not world.show_neighbourhood
	        # radius info
	        elif symbol == KEY.R:
	            world.show_radius = not world.show_radius
	        # agent wandering info
	        elif symbol == KEY.W:
	            world.show_wander = not world.show_wander
	    # obstacle menu
	    elif world.obstacle_input:
	        # spawn new obstacle
	        if symbol == KEY.N: 
	            world.obstacles.append(Obstacle(world=world))
	        # randomise the positions of the obstacles
	        elif symbol == KEY.R:
	            for obstacle in world.obstacles:
	                obstacle.randomise_position()
	    elif world.change_values:
	        if symbol == KEY.LEFT:
	            world.change_value(world.selected_variable, world.value_step, -1)
	        elif symbol == KEY.RIGHT:
	            world.change_value(world.selected_variable, world.value_step, 1)
	        elif symbol == KEY.UP:
	            world.select_variable(-1)
	        elif symbol == KEY.DOWN:
	            world.select_variable(+1)
	        elif symbol == KEY.EQUAL:		# plus and equal share a key, and plus didn't register
	            world.value_step *= 2
	        elif symbol == KEY.MINUS:
	            world.value_step /= 2
	else:
	    # open agent menu
	    if symbol == KEY.A:
	        world.new_agents = True
	        world.input_menu_open = True
	        return
        # open show info menu
	    elif symbol == KEY.I:
	        world.agent_info = True
	        world.input_menu_open = True
	        return
	    # open obstacle menu
	    elif symbol == KEY.O:
	        world.input_menu_open = True
	        world.obstacle_input = True
        # toggle on / off the ability to change values of agent variables
	    elif symbol == KEY.V:
	        world.change_values = True
	        world.input_menu_open = True


def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy

    for wall in world.walls:
    	wall.set_points()

    for obstacle in world.obstacles:
        obstacle.randomise_position()


if __name__ == '__main__':
    x = 1500
    y = 800

    # create a pyglet window and set glOptions
    win = window.Window(width=x, height=y, vsync=True, resizable=True)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # needed so that egi knows where to draw
    egi.InitWithPyglet(win)
    # prep the fps display
    fps_display = clock.ClockDisplay()
    # register key and mouse event handlers
    win.push_handlers(on_key_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(x, y)

    # add walls
    world.walls.append(Wall(world=world, side='top'))
    world.walls.append(Wall(world=world, side='bottom'))
    world.walls.append(Wall(world=world, side='left'))
    world.walls.append(Wall(world=world, side='right'))

    # add obstacles
    for n in range(6):
        world.obstacles.append(Obstacle(world=world))
    
    world.predator = Agent(world=world, mode='predator')
    world.agents.append(world.predator)
    
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
