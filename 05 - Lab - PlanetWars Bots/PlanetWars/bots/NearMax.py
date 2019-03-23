from random import choice

class NearMax(object):
    def update(self, gameinfo):
        # only send one fleet at a time
        if gameinfo.my_fleets:
            return

        # check if we should attack
        if gameinfo.my_planets and gameinfo.not_my_planets:
            #select random target and closest planet that can beat it
            dest = max(gameinfo.not_my_planets.values(), key = lambda p: 1.0 / (1 + p.num_ships))
            src = self.closest_to_dest(gameinfo.my_planets.values(), dest)

            #launch new fleet if there's enough ships
            if src.num_ships > 10:
                gameinfo.planet_order(src, dest, int(src.num_ships * 0.75))

    def closest_to_dest(self, planets, dest):
        closest = None
        dist = 0
        
        for planet in planets:
            if closest is None:
                closest = planet
                dist = planet.distance_to(dest)
            else:
                new_dist = planet.distance_to(dest)

                if new_dist < dist and planet.num_ships > dest.num_ships:
                    closest = planet
                    dist = new_dist

        return closest

