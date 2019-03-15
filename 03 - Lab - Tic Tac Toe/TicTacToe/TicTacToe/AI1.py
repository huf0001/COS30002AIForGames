import random

class AI1:
    def MakeMove(self, boardSpaces, acceptableSpaces):        
        # check for spaces that could complete a 3 in a row; such spaces can only appear for AI1 if 4 spaces are occupied (i.e. if both AIs have placed 2 marks)
        if (len(acceptableSpaces) <= 5): # if there is enough spaces filled that it is theoretically possible to win
            result = self.CheckWinningMove(acceptableSpaces, boardSpaces)

            if result is not None:
                return result

        return self.CheckNonWinningMove(acceptableSpaces)

    def CheckWinningMove(self, acceptableSpaces, boardSpaces):
        result = None
        possibleAtk3s = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [1, 4, 7], [2, 5, 8], [3, 6, 9], [1, 5, 9], [3, 5, 7]] 
        atk3Spaces = []
    
        for i in range(len(acceptableSpaces)): #for each playable space
            if (acceptableSpaces[i] not in atk3Spaces): # if it is not already designated as a winning space
                for j in range(len(possibleAtk3s)): #for each theoretical set of winning combinations
                    if (acceptableSpaces[i] in possibleAtk3s[j]): # if the playable space is in that set
                        if (self.CheckIfSetIsWinnable(acceptableSpaces[i], possibleAtk3s[j], boardSpaces)):
                            atk3Spaces.append(acceptableSpaces[i])  
                                
            if len(atk3Spaces) > 0:
                if len(atk3Spaces) is 1:
                    result = atk3Spaces[0]
                else:
                    result = atk3Spaces[random.randrange(0, len(atk3Spaces) - 1)]

        return result

    def CheckIfSetIsWinnable(self, emptySpace, winnableSet, board):
        for i in range(len(winnableSet)):                                                   # for each space in the set
            if (winnableSet[i] is not emptySpace and board[winnableSet[i] - 1] is not "X"):     # if the space specified is not an empty space AND it does not have an "X" in it
                    return False                                                            # the space is taken by the opponent and is not usable
            
        return True                                                                         # if all spaces besides emptySpace are Xs, method returns True, saying this is a winnable set

    def CheckNonWinningMove(self, acceptableSpaces):
        #pick a random space from acceptableSpaces
        if len(acceptableSpaces) is 9:
            result = acceptableSpaces[4]
        elif len(acceptableSpaces) > 1:
            result = acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]
        else:
            result = acceptableSpaces[0]
        
        return result