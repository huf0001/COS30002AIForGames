import random

class AI2:
    def MakeMove(self, boardSpaces, acceptableSpaces):
        #acceptableSpaces = []
        result = None

        #for i in range (0, len(boardSpaces)):
        #    acceptableSpaces.append(i)
        
        if len(acceptableSpaces) > 1:
            result = acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]
        else:
            result = acceptableSpaces[0]

        return result