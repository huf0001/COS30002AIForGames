'''Goal Oriented Behaviour

Created for COS30002 AI for Games, Lab,
by Clinton Woodward <cwoodward@swin.edu.au>

For class use only. Do not publically share or post this code without
permission.

Works with Python 3+

Simple decision approach.
* Choose the most pressing goal (highest insistence value)
* Find the action that fulfills this "goal" the most (ideally?, completely?)

Goal: Eat (initially = 4)
Goal: Sleep (initially = 3)

Action: get raw food (Eat -= 3)
Action: get snack (Eat -= 2)
Action: sleep in bed (Sleep -= 4)
Action: sleep on sofa (Sleep -= 2)


Notes:
* This version is simply based on dictionaries and functions.

'''

class Game:
    def __init__(self):
        self.VERBOSE = True

        # Global goals with initial values
        self.goals = {
            'Eat': 4,
            'Sleep': 3,
        }

        # Global (read-only) actions and effects
        self.actions = {
            'get raw food': { 'Eat': -3, 'Sleep': +2 },
            'get snack': { 'Eat': -1, 'Sleep': 0 },
            'sleep in bed': { 'Sleep': -4, 'Eat': +3 },
            'sleep on sofa': { 'Sleep': -2,  'Eat': + 1}
        }
  

    def print_actions(self, actions):
        print('ACTIONS:')

        for name, effects in actions.items():
            print(" * [%s]: %s" % (name, str(effects)))

        print()


    def main(self):
         players = []
         players.append(AI(self.VERBOSE, self.actions, self.goals, "AI1"))

         for player in players:
             # print(player.actions)
             # print(player.actions.items())

             #for k, v in player.actions.items():
             #    print(k,v)

             print()
             print("Player name: " + player.name)
             print()
             self.print_actions(player.actions)

             player.play_until_all_goals_zero()

         print()
         print('>> All Players Done! <<')


class AI:
    def __init__(self, verbose, actions, goals, name):
        self.VERBOSE = verbose
        self.actions = actions
        self.goals = goals
        self.name = name


    def apply_action(self, action):
        '''Change all goal values using this action. An action can change multiple
        goals (positive and negative side effects).
        Negative changes are limited to a minimum goal value of 0.
        '''
        for goal, change in self.actions[action].items():
            self.goals[goal] = max(self.goals[goal] + change, 0)


    def action_utility(self, action, goal):
        '''Return the 'value' of using "action" to achieve "goal".

        For example::
            action_utility('get raw food', 'Eat')

        returns a number representing the effect that getting raw food has on our
        'Eat' goal. Larger (more positive) numbers mean the action is more
        beneficial.
        '''
        ### Simple version - the utility is the change to the specified goal
        if goal in self.actions[action]:
            # Is the goal affected by the specified action?
            return -self.actions[action][goal]
        else:
            # It isn't, so utility is zero.
            return 0

        ### Extension
        ###
        ###     Done:
        ###         - return a higher utility for actions that don't change our goal past zero
        ###         and/or
        ###         - Add some other effects to 'actions'
        ###         - take any other (positive or negative) effects of the action into account   


    def action_side_effects(self, action, goal):
        result = 0      # default is no side effects
    
        for g in self.actions[action]:   # for each goal affected by the action
            if g is not goal:       # if it is not the current goal
                result += -self.actions[action][g]       # increment result by its effect

        return result


    def choose_action(self):
        '''Return the best action to respond to the current most insistent goal.
        '''
        assert len(self.goals) > 0, 'Need at least one goal'
        assert len(self.actions) > 0, 'Need at least one action'

        # Find the most insistent goal - the 'Pythonic' way...
        #best_goal, best_goal_value = max(goals.items(), key=lambda item: item[1])

        # ...or the non-Pythonic way. (This code is identical to the line above.)
        best_goal = None

        for key, value in self.goals.items():
            if best_goal is None or value > self.goals[best_goal]:
                best_goal = key

        if self.VERBOSE: print('BEST_GOAL:', best_goal, self.goals[best_goal])

        # Find the best (highest utility) action to take.
        # (Not the Pythonic way... but you can change it if you like / want to learn)
        best_action = None
        best_utility = None
        best_side_effects = None

        for key, value in self.actions.items():
            # Note, at this point:
            #  - "key" is the action as a string,
            #  - "value" is a dict of goal changes (see line 35)

            # Does this action change the "best goal" we need to change?
            if best_goal in value:

                # Do we currently have a "best action" to try? If not, use this one
                if best_action is None:
                    ### 1. store the "key" as the current best_action
                    best_action = key
                    ### 2. use the "action_utility" function to find the best_utility value of this best_action
                    best_utility = self.action_utility(key, best_goal)

                    best_side_effects = self.action_side_effects(key, best_goal)

                # Is this new action better than the current action?
                else:
                    ### 1. use the "action_utility" function to find the utility value of this action
                    utility = self.action_utility(key, best_goal)
                    side_effects = self.action_side_effects(key, best_goal) # side effects is sum of magnitude of all side effects
                
                    ### 2. If it's the best action to take (utility > best_utility), keep it! (utility and action)                    
                
                    ### use this code if both eating and sleeping have -ve side effects of great enough magnitude that neither would allow for comparison based on mitigating side effects because
                    ### the code is too concerned with managing the dominant goal
                    #if utility > 0 and side_effects >= 0 and side_effects > best_side_effects: # if side effects are none / beneficial, and better than best_side_effects
                    #    best_action = key
                    #    best_utility = action_utility(key, best_goal)
                    #    best_side_effects = action_side_effects(key, best_goal)
                    #el
                    if (utility >= self.goals[best_goal] and best_utility >= self.goals[best_goal]) or (utility is best_utility): # if both beat goal or are equal
                        if side_effects > best_side_effects:    
                            # print("Both beat goal; judging by side effects")
                            best_action = key
                            best_utility = self.action_utility(key, best_goal)
                            best_side_effects = self.action_side_effects(key, best_goal)

                    elif utility > best_utility: # if utility beats best_utility
                        # print("Updating best_utility based on maximising intended effect")
                        best_action = key
                        best_utility = self.action_utility(key, best_goal)
                        best_side_effects = self.action_side_effects(key, best_goal)

        # Return the "best action"
        return best_action


    def play_until_all_goals_zero(self):
        HR = '-'*40
        playing = True

        print('>> Start <<')
        print(HR)

        while playing:
            print('GOALS:', self.goals)

            # What is the best action
            self.action = self.choose_action()
            print('BEST ACTION:', self.action)

            # Apply the best action
            self.apply_action(self.action)
            print('NEW GOALS:', self.goals)

            # Stop?
            if all(value == 0 for goal, value in self.goals.items()):
                playing = False

            print(HR)

            input()

        # finished
        print('>> Done! <<')


Game().main()

