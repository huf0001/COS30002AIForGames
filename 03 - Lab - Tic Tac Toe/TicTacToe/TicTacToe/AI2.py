import random

class AI2:
    def MakeMove(self, boardSpaces, acceptableSpaces):
        result = None
        
        if (len(acceptableSpaces) < 7):                                   # 2s and 3s in a row will only appear after AI2 has had its first turn
            result = self.CheckAttackableSpaces(boardSpaces, acceptableSpaces)

            if result is not None:
                return result

        return self.CheckOtherSpaces(acceptableSpaces)
    
    # find and choose an attackable space
    def CheckAttackableSpaces(self, boardSpaces, acceptableSpaces):
        print ("AI2 checking attackable Spaces . . .")
        atkSpaces = []
        multiAtkSpaces = []
        possibleAtks = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [1, 4, 7], [2, 5, 8], [3, 6, 9], [1, 5, 9], [3, 5, 7]] 
        unobstructed = True
        adjacentO = False

        # finding viable attacking spaces
        for i in range(len(acceptableSpaces)):                              # for each space in acceptableSpaces
            for j in range(len(possibleAtks)):                              # for each set of 3 in a row in atkSpaces
                if acceptableSpaces[i] in possibleAtks[j]:                  # if the acceptableSpace is in that row of atkSpaces
                    for k in range(len(possibleAtks[j])):                   # for each space in that row of atkSpaces
                        if boardSpaces[possibleAtks[j][k] - 1] is "X":      # if that space on the board is an "X"
                            unobstructed = False                            # that row is not an unobstructed attacking row
                        elif boardSpaces[possibleAtks[j][k] - 1] is "O":    # else if that row has an "O"
                            adjacentO = True                                # that row is a potential attacking row

                    if unobstructed is True and adjacentO is True:
                        if acceptableSpaces[i] not in atkSpaces:
                            atkSpaces.append(acceptableSpaces[i])
                        elif acceptableSpaces[i] not in multiAtkSpaces:
                            multiAtkSpaces.append(acceptableSpaces[i])

                    unobstructed = True
                    adjacentO = False
        
        # choosing an attacking space
        if len(multiAtkSpaces) > 0 and random.randrange(0, 1) > 0.5:        # 50% chance of going with a multi-atk space if they're available
            if len(multiAtkSpaces) is 1:
                return multiAtkSpaces[0]
            else:
                return multiAtkSpaces[random.randrange(0, len(multiAtkSpaces) - 1)]
        elif len(atkSpaces) > 0:                                            # otherwise pick any random attackable space
            print("AI2 found attackable spaces")
            if len(atkSpaces) is 1:
                return atkSpaces[0]
            else:
                return atkSpaces[random.randrange(0, len(atkSpaces) - 1)]
        else:
            return None
    
    # choosing a mostly random space
    def CheckOtherSpaces(self, acceptableSpaces):
        if len(acceptableSpaces) >= 8:
            temp = [0, 2, 6, 8]                                                           # start in a corner space
            return acceptableSpaces[temp[random.randrange(0, 3)]]
        elif len(acceptableSpaces) > 1:
            return acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]       # mark a random available space
        else:
            return acceptableSpaces[0]                                                    # mark the last space available