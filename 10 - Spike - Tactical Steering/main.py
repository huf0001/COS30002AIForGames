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
from obstacle import Obstacle

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


# def on_mouse_press(x, y, button, modifiers):
#     if button == 1:  # left
#         world.target = Vector2D(x, y)


def on_key_press(symbol, modifiers):
    # indicate that you want to create new agents
    # if symbol == KEY.A:
    #     world.new_agents = True
    #     return
    # spawning new agents
    # elif world.new_agents and symbol in NUM_KEYS:
    #     loop = NUM_KEYS[symbol]

    #     while loop > 0:
    #         world.agents.append(Agent(world=world, mode=world.agent_mode))
    #         loop -= 1
    # apply new movement behaviour
    # elif symbol in AGENT_MODES:
    #     mode = AGENT_MODES[symbol]

    #     if mode == 'pursuit' and len(world.agents) == 1:
    #         return

    #     world.agent_mode = mode

    #     for agent in world.agents:
    #         agent.mode = mode

    #     if mode == 'pursuit':
    #         world.agents[0].mode = 'flee'
    #         world.evader = world.agents[0]

    # generate new evading agents   #el
    if symbol == KEY.E:
        world.agents.append(Agent(world=world, scale=world.agent_scale, mode='hide', speed_limiter=3, radius=world.agent_radius))
        world.evaders.append(world.agents[len(world.agents) - 1])
    # toggle friction on and off
    elif symbol == KEY.F:
        for agent in world.agents:
            agent.applying_friction = not agent.applying_friction
    # add wandering hunter agent
    elif symbol == KEY.H:        
        world.agents.insert(0, Agent(world=world, scale=world.agent_scale, mode='hunt', speed_limiter=5, radius=world.agent_radius))
        world.hunters.append(world.agents[0])
    # Toggle debug force line info on the agent
    elif symbol == KEY.I:
        for agent in world.agents:
            agent.show_info = not agent.show_info
    # create a new obstacle
    elif symbol == KEY.O:
        world.obstacles.append(Obstacle(world=world))
    # Pause / unpause the game
    elif symbol == KEY.P:
        world.paused = not world.paused
    # randomise the positions of the obstacles
    elif symbol == KEY.R:
        for obstacle in world.obstacles:
            obstacle.randomise_position()
    # randomise the paths of the agents
    # elif symbol == KEY.R:
    #     for agent in world.agents:
    #         agent.randomise_path()
    # randomise the positions of the obstacles in the current world space

    # world.new_agents = False


def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy

    for obstacle in world.obstacles:
        obstacle.randomise_position()


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
    #win.push_handlers(on_mouse_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(2000, 1000)
    # add obstacles
    for n in range(6):
        world.obstacles.append(Obstacle(world=world))
    
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
