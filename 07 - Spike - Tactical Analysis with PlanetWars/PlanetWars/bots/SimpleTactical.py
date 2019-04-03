from enum import Enum
from bots import BotEvents 

class State(Enum):
    Attacking = 1
    Growing = 2
    Waiting = 3

class SimpleTactical(object):
    def __init__(self):
        self.bot_events = BotEvents.BotEvents()
        self.state = State.Growing
        self.leave_in_reserve = 5
    
    def update(self, gameinfo):     
        # only send one fleet at a time
        if gameinfo.my_fleets:
            return

        # check if we should attack
        if gameinfo.my_planets and gameinfo.not_my_planets:
            self.check_state(gameinfo)
            src = None
            dest = None
            dests = []
            
            #Attacking: getting enemy's highly productive planets
            if self.state is State.Attacking:
                print("Attacking")
                dests = self.rank_by_productive(gameinfo.enemy_planets.values())

                for p in dests:
                    src = self.find_closest_to(self.find_all_stronger(gameinfo.my_planets.values(), p.num_ships + (2 * self.leave_in_reserve)), p)
                    
                    if src is not None:
                        dest = p
                        break

                if src is not None:
                    print("Found source")
                    self.dispatch_fleet(gameinfo, src, dest, int(src.num_ships - self.leave_in_reserve))
                else:
                    print("No known suitable source")
            
            #Growing: getting any highly productive planet, enemy or not
            elif self.state is State.Growing:
                print("Growing")
                dests = self.rank_by_productive(gameinfo.not_my_planets.values())

                for p in dests:
                    src = self.find_closest_to(self.find_all_stronger(gameinfo.my_planets.values(), p.num_ships + (2 * self.leave_in_reserve)), p)
                    
                    if src is not None:
                        dest = p
                        break

                if src is not None:
                    print("Found source")
                    self.dispatch_fleet(gameinfo, src, dest, int(src.num_ships - self.leave_in_reserve))
                else:
                    print("No known suitable source")            

            #Waiting: "And neutral jing, when you do nothing." - King Bumi
            elif self.state is State.Waiting:
                print("Waiting...")

            #Other: an error has occured
            else:
                print("Error: AI ComplexTactical.state has not been asigned a valid value")
                return
    
    def check_state(self, gameinfo):        
        # states:
        #   - Attacking: getting enemy's highly productive planets
        #   - Growing: getting any highly productive planet, enemy or not
        #   - Waiting: "And neutral jing, when you do nothing." - King Bumi
        
        if len(gameinfo.not_my_planets) is 0:
            self.State = State.Waiting
            print("Can't see any planets to attack")
        elif len(gameinfo.enemy_planets) is 0:
            self.state = State.Growing
        elif len(gameinfo.enemy_planets) <= 5 and len(gameinfo.my_planets) > len(gameinfo.enemy_planets) + 5:
            self.state = State.Attacking
        elif len(gameinfo.my_planets) < len(gameinfo.enemy_planets):
            self.state = State.Growing
        else:
            self.state = State.Attacking

    def find_most_productive(self, planets):
        most_productive = None
        best_production = 0
        
        for planet in planets:
            if planet.growth_rate > best_production:
                most_productive = planet
                best_production = planet.growth_rate

        return most_productive

    def rank_by_productive(self, planets):
        sorted = []

        for p in planets:
            sorted.append(p)

        sorted.sort(key = lambda x: x.growth_rate, reverse = True)
        return sorted

    def find_all_stronger(self, planets, target):
        stronger = []
        
        for planet in planets:
            if planet.num_ships > target:
                stronger.append(planet)

        return stronger

    def find_closest_to(self, planets, dest):
        closest = None
        dist = 9999999999

        for planet in planets:
            if planet.distance_to(dest) < dist:
                closest = planet
                dist = planet.distance_to(dest)

        return closest

    def dispatch_fleet(self, gameinfo, src, dest, num_ships):
        gameinfo.planet_order(src, dest, num_ships)
        self.bot_events.new_fleet(src, dest)