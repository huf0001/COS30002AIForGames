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

    
            # check for spaces that could block a 3 in a row. Only check if you cannot create a 3 in a row. These moves will also only appear for AI1 if 4 space are occupied (i.e. if both AIs have placed 2 marks)
            #    else:
            #        potentialAtk3 = False
            #
            #        for i in acceptableSpaces:
            #            for j in possibleAtk3s:
            #                for k in possibleAtk3s[j]:
            #                    if not potentialAtk3:
            #                        if possibleAtk3s[j][k] not in acceptableSpaces and boardSpaces[possibleAtk3s[j][k] - 1] is not "X":
            #                            potentialAtk3 = True
            

        # check for spaces that could complete a 2 in a row. Such spaces can only appear for AI1 if 2 spaces are occupied (i.e. if both AIs have placed 1 mark)
        #if len(acceptableSpaces) <= 7:
        #    print()

        # check for spaces that could block the opponent from completing a 3 in a row and make a 2 in a row
        #if len(blockSpaces) > 0 and len(atk2Spaces) > 0:
        #    for i in range(0, len(blockSpaces) - 1):
        #        if blockSpaces[i] in atk2Spaces:
        #            blockAndAtk2Spaces.append(blockSpaces[i])

        # if len(blockAndAtk2Spaces) > 0:
            # check for spaces that could block the opponent from completing a 3 in a row and make 2 2's in a row

            # else pick a random space from blockAndAtk2Spaces

        # else if len(blockSpaces) > 0:
            # pick a random space from blockSpaces

        # else if (atk2Spaces) > 0:
            # pick a random space from atk2Spaces

        # else

        
        # note: best move when the board is empty, or multiple block / atk2 / blockAndAtk2 spaces are available, is space no. 5 (the centre space)