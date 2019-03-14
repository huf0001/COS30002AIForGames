import AI1 as ai1Module
import AI2 as ai2Module

class TicTacToeGame:
    def __init__(self):
        self.ai1 = ai1Module.AI1()
        self.ai2 = ai2Module.AI2()
        
        self.input = 0
        self.currentAI = 0
        self.boardSpaces = [" ", " ", " ", " ", " ", " ", " ", " ", " "]
        self.acceptableBoardSpaces = [1, 2, 3, 4, 5, 6, 7, 8, 9]

        # self.finished = False
        self.gameOver = False
        self.winner = None

    def GameLoop(self):
        '''
            Display the space numbers on the game board to screen
            print('    1 | 2 | 2')
            print('   -----------')
            print('    4 | 5 | 6')
            print('   -----------')
            print('    7 | 8 | 9')
        '''
        
        while not self.gameOver:
            self.input = self.GatherInput(self.currentAI, self.boardSpaces, self.acceptableBoardSpaces)

            self.Update(self.currentAI, self.input, self.boardSpaces, self.acceptableBoardSpaces)

            self.Render(self.currentAI, self.boardSpaces)

    def GatherInput(self, ai, boardSpaces, acceptableSpaces):
        # increment which AI's turn it is
        if self.currentAI == 1:
            self.currentAI = 2
        else:
            self.currentAI = 1

        # check which AI's turn it is and get input from AI whose turn it is
        if ai == 1:
            return self.ai1.MakeMove(boardSpaces, acceptableSpaces)
        else:
            return self.ai2.MakeMove(boardSpaces, acceptableSpaces)

    def Update(self, ai, input, boardSpaces, acceptableSpaces):        
        # update game state based on which AI's turn it is and what move they made
        boardSpaces[input - 1] = self.GetAIMark(ai)
        acceptableSpaces.remove(input)

        # check if an AI has won the game
        self.CheckGameWon(boardSpaces)
            
        # check if all spaces on the board have been filled
        if len(acceptableSpaces) == 0:
            self.gameOver = True

    def GetAIMark(self, ai):
        if ai == 1:
            return "X"
        else:
            return "O"

    def Render(self, ai, boardSpaces):
        # display the results of the AI's move
        print("AI no. " + str(ai) + " placed an " + self.GetAIMark(ai) + " in space number " + str(self.input))

        # Display the current game board to screen
        print('    %s | %s | %s' % tuple(boardSpaces[:3]))
        print('   -----------')
        print('    %s | %s | %s' % tuple(boardSpaces[3:6]))
        print('   -----------')
        print('    %s | %s | %s' % tuple(boardSpaces[6:]))

        if self.gameOver:
            print("Game Over!")
            if self.winner != None:
                print("The winner is " + self.winner)
        else:
            input("Press enter to continue")

    def CheckGameWon(self, boardSpaces):
        #local variables
        ai1Spaces = []
        ai2Spaces = []
        
        # check which spaces have been occupied by which player
        for i in range (0, len(boardSpaces) - 1):
            if boardSpaces[i] == "X":
                ai1Spaces.append(str(i + 1))
            elif boardSpaces[i] == "O":
                ai2Spaces.append(str(i + 1))

        if self.FoundWinningCombination(ai1Spaces):
            self.winner = "AI1"
            self.gameOver = True
        elif self.FoundWinningCombination(ai2Spaces):
            self.winner = "AI2"
            self.gameOver = True

    def FoundWinningCombination(self, aiSpaces):
        # check for winning combinations
        # syntax: if all(elem in SUPERSET for elem in SUBSET):
        if all(elem in aiSpaces for elem in ["1", "2", "3"]):
                return True
        elif all(elem in aiSpaces for elem in ["4", "5", "6"]):
                return True
        elif all(elem in aiSpaces for elem in ["7", "8", "9"]):
                return True
        elif all(elem in aiSpaces for elem in ["1", "4", "7"]):
                return True
        elif all(elem in aiSpaces for elem in ["2", "5", "8"]):
                return True
        elif all(elem in aiSpaces for elem in ["3", "6", "9"]):
                return True
        elif all(elem in aiSpaces for elem in ["1", "5", "9"]):
                return True
        elif all(elem in aiSpaces for elem in ["3", "5", "7"]):
                return True
        else:
            return False
