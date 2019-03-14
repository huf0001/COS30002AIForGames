import random

class AI1:
    def MakeMove(self, boardSpaces, acceptableSpaces):
        # result = None
        atk3Spaces = []
        blockSpaces = []
        atk2Spaces = []
        blockAndAtk2Spaces = []

        potentialAtk3 = True

        possibleAtk3s = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [1, 4, 7], [2, 5, 8], [3, 6, 9], [1, 5, 9], [3, 5, 7]]

        # for i in acceptableSpaces:
            # CheckAtk3Spaces(acceptableSpaces[i], boardSpaces)
            # CheckBlockSpaces(acceptableSpaces[i], boardSpaces)
            # CheckAtk2Spaces(acceptableSpaces[i], boardSpaces)
        
        # check for spaces that could complete a 3 in a row; such spaces can only appear for AI1 if 4 spaces are occupied (i.e. if both AIs have placed 2 marks)
        if len(acceptableSpaces) <= 5:
            print()
            #for i in acceptableSpaces:
            #    if acceptableSpaces[i] not in atk3Spaces:
            #        for j in range (0, len(possibleAtk3s) - 1):
            #            if acceptableSpaces[i] in possibleAtk3s[j]:
            #                for k in possibleAtk3s[j]:
            #                    if potentialAtk3 == True:
            #                        if possibleAtk3s[j][k] is not acceptableSpaces[i] and boardSpaces[possibleAtk3s[j][k] - 1] is not "X":
            #                            potentialAtk3 = False
            #
            #                if potentialAtk3 == True:
            #                    atk3Spaces.append(acceptableSpaces[i])
            #                else:
            #                    potentialAtk3 = True

            # check for spaces that could block a 3 in a row. Only check if you cannot create a 3 in a row. These moves will also only appear for AI1 if 4 space are occupied (i.e. if both AIs have placed 2 marks)
            
            #    if len(atk3Spaces) > 0:
            #        if len(atk3Spaces) == 1:
            #            return atk3Spaces[0]
            #        else:
            #            return atk3Spaces[random.randrange(0, len(atk3Spaces) - 1)]
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
        if len(acceptableSpaces) <= 7:
            print()
                       

            

        # check for spaces that could block the opponent from completing a 3 in a row and make a 2 in a row
        if len(blockSpaces) > 0 and len(atk2Spaces) > 0:
            for i in range(0, len(blockSpaces) - 1):
                if blockSpaces[i] in atk2Spaces:
                    blockAndAtk2Spaces.append(blockSpaces[i])

        # if len(blockAndAtk2Spaces) > 0:
            # check for spaces that could block the opponent from completing a 3 in a row and make 2 2's in a row

            # else pick a random space from blockAndAtk2Spaces

        # else if len(blockSpaces) > 0:
            # pick a random space from blockSpaces

        # else if (atk2Spaces) > 0:
            # pick a random space from atk2Spaces

        # else
        #pick a random space from acceptableSpaces
        return acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]
        
        # note: best move when the board is empty, or multiple block / atk2 / blockAndAtk2 spaces are available, is space no. 5 (the centre space)

        # return result
        
        # if len(acceptableSpaces) > 1:
        #     result = acceptableSpaces[random.randrange(0, len(acceptableSpaces) - 1)]
        # else:
        #     result = acceptableSpaces[0]