import random

class AI2:
    def MakeMove(self, boardSpaces, acceptableSpaces):
        result = None
        
        if (len(acceptableSpaces) < 7):
            result = self.CheckAttackableSpaces(boardSpaces, acceptableSpaces)

            if result is not None:
                return result

        return self.CheckOtherSpaces(acceptableSpaces)
    
    def CheckAttackableSpaces(self, boardSpaces, acceptableSpaces):
        print ("AI2 checking attackable Spaces . . .")
        atkSpaces = []
        multiAtkSpaces = []
        possibleAtks = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [1, 4, 7], [2, 5, 8], [3, 6, 9], [1, 5, 9], [3, 5, 7]] 
        possibleAtk = True
        # check for spaces that could complete a 2 in a row.
            # conditions:
            #   there is a space marked by AI2 in a row/column/diagonal
            #   there is an empty space in that row/diagonal/column (found by verifying that it is in acceptableSpaces)
            #   AI1 has not marked a space in that row/column/diagonal
        for i in range(len(acceptableSpaces)):
            for j in range(len(possibleAtks)):
                if acceptableSpaces[i] in possibleAtks[j]:
                    for k in range(len(possibleAtks[j])):
                        if boardSpaces[possibleAtks[j][k] - 1] is "X":
                            possibleAtk = False

                    if possibleAtk is True:
                        if acceptableSpaces[i] not in atkSpaces:
                            atkSpaces.append(acceptableSpaces[i])
                        elif acceptableSpaces[i] not in multiAtkSpaces:
                            multiAtkSpaces.append(acceptableSpaces[i])
                    else:
                        possibleAtk = True
        # spaces that are not viable attackable spaces are finding their way into atkSpaces and multiAtkSpaces. They shouldn't do that. Error in code would be above in the logic for selecting if a space gets added to atkSpaces
        if len(multiAtkSpaces) > 0 and random.randrange(0, 1) > 0.5:
            if len(multiAtkSpaces) is 1:
                return multiAtkSpaces[0]
            else:
                return multiAtkSpaces[random.randrange(0, len(multiAtkSpaces) - 1)]
        elif len(atkSpaces) > 0:
            print("AI2 found attackable spaces")
            if len(atkSpaces) is 1:
                return atkSpaces[0]
            else:
                return atkSpaces[random.randrange(0, len(atkSpaces) - 1)]
        else:
            return None

    def CheckOtherSpaces(self, acceptableSpaces):
        if len(acceptableSpaces) == 9:
            temp = [0, 2, 6, 8]                                                           # start in a corner space
            return acceptableSpaces[temp[random.randrange(0, 3)]]
        elif len(acceptableSpaces) > 1:
            return acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]       # mark a random available space
        else:
            return acceptableSpaces[0]                                                    # mark the last space available