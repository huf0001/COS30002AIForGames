import random

class AI2:
    def MakeMove(self, boardSpaces, acceptableSpaces):
        #acceptableSpaces = []
        result = None

        #for i in range (0, len(boardSpaces)):
        #    acceptableSpaces.append(i)
        
        if len(acceptableSpaces) == 9:
            temp = [0, 2, 6, 8]   
            result = acceptableSpaces[temp[random.randrange(0, 3)]]
        elif len(acceptableSpaces) > 1:
            result = acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]
        else:
            result = acceptableSpaces[0]

        return result