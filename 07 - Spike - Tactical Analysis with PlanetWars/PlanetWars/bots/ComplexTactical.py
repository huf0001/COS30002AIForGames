from enum import Enum
from bots import BotEvents 
from pyglet import window

class State(Enum):
    Attacking = 1
    Defending = 2
    Growing = 3
    Raiding = 4
    Sabotaging = 5
    Waiting = 6

class enemy_fleet():
        def __init__(self, src, dest):
            self.src = src
            self.dest = dest

class ComplexTactical(object):    
    def __init__(self):
        self.bot_events = BotEvents.BotEvents()
        self.state = State.Growing
        self.leave_in_reserve = 5
        self.best_planets = []
        self.new_enemy_fleet = None

        def on_new_fleet(src, dest):
            self.new_enemy_fleet = enemy_fleet(src, dest)

        self.bot_events.push_handlers(on_new_fleet)
    
    def update(self, gameinfo):     
        # only send one fleet at a time
        if not gameinfo.my_fleets:
            # check if we should make a move
            if gameinfo.my_planets and gameinfo.not_my_planets:
                self.best_planets = self.get_best_planets(gameinfo.my_planets.values())
                self.check_state(gameinfo)
           
                if self.state is State.Attacking:  
                    self.attack(gameinfo)
            
                elif self.state is State.Defending:
                    self.defend(gameinfo)

                elif self.state is State.Growing:
                    self.grow(gameinfo)

                elif self.state is State.Raiding:
                    self.raid(gameinfo)

                elif self.state is State.Waiting:
                    print("Can't see any planets to attack. Waiting...")

                else:
                    print("Error: AI ComplexTactical.state has not been asigned a valid value")
                    print("Forcing AI ComplexTactical to grow")
                    self.state = State.Growing
                    self.grow(gameinfo)

        self.new_enemy_fleet = None

    def check_state(self, gameinfo):        
        # states:
        #   - Attacking: getting enemy's highly productive planets
        #   - Defending: reinforcing my most productive planets
        #   - Growing: getting any highly productive planet, enemy or not
        #   - Raiding: attacking a planet the enemy just left vulnerable
        #   - Sabotaging: attacking an average-production enemy planet from a low-production planet with lots of ships
        #   - Waiting: "And neutral jing, when you do nothing." - King Bumi
        
        if len(gameinfo.not_my_planets) is 0:
            self.state = State.Waiting
        elif len(gameinfo.my_planets) >= 10 and not self.best_are_defended(gameinfo.my_planets.values(), self.best_planets):
            self.state = State.Defending
        elif len(gameinfo.enemy_planets) is 0:
            self.state = State.Growing
        elif len(gameinfo.enemy_planets) <= 5 and len(gameinfo.my_planets) > len(gameinfo.enemy_planets) + 5:
            self.state = State.Attacking
        elif len(gameinfo.my_planets) < len(gameinfo.enemy_planets):
            self.state = State.Growing
        else:
            self.state = State.Attacking
        
        if self.state is State.Attacking or self.state is State.Growing:
            if self.defend_enemy_target():
                self.state = State.Defending
            elif self.new_enemy_fleet is not None:            
                self.state = State.Raiding

#----------------------------------------------------------------------------------------------------------------------------------------------

    #Attacking: getting enemy's highly productive planets
    def attack(self, gameinfo):
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
            return
        else:
            self.state = State.Sabotaging
            self.sabotage(gameinfo)

    #Defending: reinforcing my most productive planets
    def defend(self, gameinfo):
        print("Reinforcing high-production planets")

        if self.defend_enemy_target():
            dest = self.new_enemy_fleet.dest

            if dest not in self.best_planets:
                self.best_planets.append(dest)
        else:
            dest = self.find_weakest(self.best_planets)

        src = self.find_strongest_excluding_best(gameinfo.my_planets.values(), self.best_planets)
        self.dispatch_fleet(gameinfo, src, dest, int(src.num_ships / 2))

    #Growing: getting any highly productive planet, enemy or not
    def grow(self, gameinfo):
        print("Growing")
        dest = None
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
            self.state = State.Sabotaging
            self.sabotage(gameinfo)
    
    #Raiding: attacking a planet the enemy just left vulnerable
    def raid(self, gameinfo):
        print("RAIDING!!!")
        stronger_on_combat = self.find_all_stronger_on_combat(gameinfo.my_planets.values(), self.new_enemy_fleet.src, self.new_enemy_fleet.src.num_ships + (2 * self.leave_in_reserve))
        src = self.find_closest_to(stronger_on_combat, self.new_enemy_fleet.src)
                    
        if src is not None:
            self.dispatch_fleet(gameinfo, src, self.new_enemy_fleet.src, int(src.num_ships - self.leave_in_reserve))
        else:
            self.grow(gameinfo)

    #Sabotaging: attacking an average-production enemy planet from a low-production planet with lots of ships
    def sabotage(self, gameinfo):
        print("Nope, sabotaging instead . . .")
        src = self.find_least_productive_but_strongest(gameinfo.my_planets.values())
        dest = self.find_median_production(gameinfo.enemy_planets.values())
        self.dispatch_fleet(gameinfo, src, dest, int(src.num_ships / 2))

#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def defend_enemy_target(self):
        if self.new_enemy_fleet is not None:
            if self.new_enemy_fleet.dest in self.best_planets:
                defender = self.closest_of_strongest(self.new_enemy_fleet.dest, gameinfo.my_planets.values())
                    
                if defender is not None:
                    if self.new_enemy_fleet.dest.distance_to(defender) < self.new_enemy_fleet.dest.distance_to(self.new_enemy_fleet.src):
                        return True
        return False

    def find_closest_to(self, planets, dest):
        closest = None
        dist = 9999999999

        for p in planets:
            if p.distance_to(dest) < dist and p is not dest:
                closest = p
                dist = p.distance_to(dest)

        return closest

    def find_closest_of_strongest(self, dest, planets):
        best = []
        sorted = self.rank_by_strength(planets)

        for i in range(0, 5):
            best.append(sorted[i])

        return find_closest_to(best, dest)

    def find_least_productive_but_strongest(self, planets):
        sorted = []
        least_productive = []

        for p in planets:
            sorted.append(p)

        sorted.sort(key = lambda x: x.growth_rate)

        if len(sorted) > 5:
            for i in range(0, 5):
                least_productive.append(sorted[i])
        else:
            least_productive = sorted

        return self.find_strongest(least_productive)

    def find_median_production(self, planets):
        if len(planets) is 0:
            return None
        
        sorted = []

        for p in planets:
            sorted.append(p)

        if len(sorted) == 1:
            return sorted[0]
        else:
            sorted.sort(key = lambda x: x.growth_rate)
            middle = int(len(sorted) / 2)

            if middle < 0:
                middle = 0
            elif middle >= len(sorted):
                middle = len(sorted) - 1

            return sorted[middle]

    #def find_most_productive(self, planets):
    #    most_productive = None
    #    best_production = 0
        
    #    for p in planets:
    #        if p.growth_rate > best_production:
    #            most_productive = p
    #            best_production = p.growth_rate

    #    return most_productive

    def rank_by_productive(self, planets):
        sorted = []

        for p in planets:
            sorted.append(p)

        sorted.sort(key = lambda x: x.growth_rate, reverse = True)

        return sorted

    def get_best_planets(self, planets):
        best = []
        sorted = self.rank_by_productive(planets)

        if len(sorted) <= 5:
            return sorted
        else:
            for i in range(0, 5):
                best.append(sorted[i])

            return best

    def best_are_defended(self, my_planets, best_planets):
        total = 0
        average = 0

        for p in my_planets:
            total += p.num_ships

        average = total / len(my_planets)

        for p in best_planets:
            if p.num_ships < average:
                return False

        return True

    def find_weakest(self, planets):
        weakest = None
        strength = 999999999999999999999
        
        for p in planets:
            if p.num_ships < strength:
                weakest = p
                strength = p.num_ships

        return weakest

    def rank_by_strength(self, planets):
        sorted = []

        for p in planets:
            sorted.append(p)

        sorted.sort(key = lambda x: x.growth_rate, reverse = True)

        return sorted

    def find_all_stronger(self, planets, target):
        stronger = []
        
        for p in planets:
            if p.num_ships > target:
                stronger.append(p)

        return stronger

    def find_all_stronger_on_combat(self, planets, dest, target_ships):
        stronger_on_combat = []
        stronger = self.find_all_stronger(planets, target_ships)

        for p in stronger:
            if p.num_ships > 5 + (dest.growth_rate * p.distance_to(dest)) and p is not dest:
                stronger_on_combat.append(p)

        return stronger_on_combat

    ###def find_just_stronger(self, planets, target):
    ###    best = None
    ###    strength = 999999999
    ###    target += (2 * self.leave_in_reserve)

    ###    for p in planets:
    ###        if p.num_ships >= target and p.num_ships < strength:
    ###            best = p
    ###            strength = planet.num_ships

    ###    return best

    def find_strongest_excluding_best(self, my_planets, best_planets):
        planets = []

        for p in my_planets:
            if p not in best_planets:
                planets.append(p)

        return self.find_strongest(planets)

    def find_strongest(self, planets):
        strongest = None
        strength = 0
        
        for p in planets:
            if p.num_ships > strength:
                strongest = p
                strength = p.num_ships

        return strongest

    def dispatch_fleet(self, gameinfo, src, dest, num_ships):
        gameinfo.planet_order(src, dest, num_ships)
        self.bot_events.new_fleet(src, dest)
