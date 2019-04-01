from random import choice
from pyglet.event import EventDispatcher
from bots import BotEvents 

class Naive(object):
    def __init__(self):
        self.bot_events = BotEvents.BotEvents()

    def update(self, gameinfo):
        # only send one fleet at a time
        if gameinfo.my_fleets:
            return

        # check if we should attack
        if gameinfo.my_planets and gameinfo.not_my_planets:
            #select random target and destination
            src = choice(list(gameinfo.my_planets.values()))
            
            target = choice([1, 2, 3])

            if target is 1 and len(gameinfo.neutral_planets) > 0:
                dest = choice(list(gameinfo.neutral_planets.values()))
            elif target is 2 and len(gameinfo.enemy_planets) > 0:
                dest = choice(list(gameinfo.enemy_planets.values()))
            else:
                dest = choice(list(gameinfo.not_my_planets.values()))

            #launch new fleet if there's enough ships
            if src.num_ships > 10:
                gameinfo.planet_order(src, dest, int(src.num_ships * 0.75))
                self.bot_events.new_fleet(src, dest)