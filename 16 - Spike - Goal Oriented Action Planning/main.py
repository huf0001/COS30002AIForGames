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
from wall import Wall
from path import Path

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
    # force display of avoidance ranges
    if symbol == KEY.A:
        for agent in world.agents:
            agent.show_avoidance = not agent.show_avoidance
    # toggle walls on and off
    # elif symbol == KEY.B:
    #     world.walls_enabled = not world.walls_enabled
    # Toggle debug force line info on the agent
    elif symbol == KEY.I:
        for agent in world.agents:
            agent.show_info = not agent.show_info
    # create a new obstacle of obstacles are enabled
    # elif symbol == KEY.N and world.obstacles_enabled:
    #     world.obstacles.append(Obstacle(world=world))
    # toggle obstacles on and off
    # elif symbol == KEY.O:
    #     world.obstacles_enabled = not world.obstacles_enabled
    # Pause / unpause the game
    elif symbol == KEY.P:
        world.paused = not world.paused
    # randomise the positions of the obstacles
    # elif symbol == KEY.R:
    #     for obstacle in world.obstacles:
    #         obstacle.randomise_position()
    # respawn dead shooter
    elif symbol == KEY.S:
        if world.shooter == None:
            world.shooter = Agent(world=world, agent_type='shooter', weapon='Rifle')
            world.agents.append(world.shooter)
            world.shooter.path = Path(num_pts=9, looped=True)
            world.shooter.heading = Vector2D(0,1)
            world.shooter.side = world.shooter.heading.perp()
            world.shooter.path.recreate_preset_path(maxx=world.cx, maxy=world.cy)
            world.shooter.update_hunt_dist()
    # respawn dead target
    elif symbol == KEY.T:
        if world.target == None:
            world.target = Agent(world=world, agent_type='target')
            world.agents.append(world.target)
            world.target.heading = Vector2D(0,1)
            world.target.side = world.target.heading.perp()
    # scroll through shooter weapons
    elif symbol == KEY.W:
        world.shooter.next_weapon()

def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy

    for obstacle in world.obstacles:
        obstacle.randomise_position()

    for wall in world.walls:
        wall.set_points()

    world.set_agents(cx, cy)


if __name__ == '__main__':
    x = 1600
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

    # add obstacles
    for n in range(6):
        world.obstacles.append(Obstacle(world=world))

    # add walls
    world.walls.append(Wall(world=world, side='top'))
    world.walls.append(Wall(world=world, side='bottom'))
    world.walls.append(Wall(world=world, side='left'))
    world.walls.append(Wall(world=world, side='right'))

    # add agents
    world.set_agents(world.cx, world.cy)
    
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
