from random import choice

class Max(object):
    def update(self, gameinfo):
        # only send one fleet at a time
        if gameinfo.my_fleets:
            return

        # check if we should attack
        if gameinfo.my_planets and gameinfo.not_my_planets:
            #select random target and destination
            dest = max(gameinfo.not_my_planets.values(), key = lambda p: 1.0 / (1 + p.num_ships))
            src = choice(list(gameinfo.my_planets.values()))
            #dest = self.select_planet(gameinfo.not_my_planets.values())

            #launch new fleet if there's enough ships
            if src.num_ships > 10:
                gameinfo.planet_order(src, dest, int(src.num_ships * 0.75))