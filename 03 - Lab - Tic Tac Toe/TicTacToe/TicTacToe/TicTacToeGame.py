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
        result = None
        
        while not self.gameOver:
            self.input = self.GatherInput(self.boardSpaces, self.acceptableBoardSpaces)

            self.Update(self.currentAI, self.input, self.boardSpaces, self.acceptableBoardSpaces)

            self.Render(self.currentAI, self.boardSpaces)
        
        result = input("Play again? (Y/N): ")
        if result in ["Y", "y", "Yes", "yes"]:
            return True
        else:
            return False    

    def GatherInput(self, boardSpaces, acceptableSpaces):
        # increment which AI's turn it is, and let it have its turn
        if self.currentAI == 1:
            self.currentAI = 2
            return self.ai2.MakeMove(boardSpaces, acceptableSpaces)
        else:
            self.currentAI = 1
            return self.ai1.MakeMove(boardSpaces, acceptableSpaces)

    def Update(self, ai, input, boardSpaces, acceptableSpaces):        
        # update game state based on which AI's turn it is and what move they made
        boardSpaces[input - 1] = self.GetAIMark(ai)
        acceptableSpaces.remove(input)

        # check if an AI has won the game
        self.CheckGameWon(boardSpaces, ai)
            
        # check if all spaces on the board have been filled
        if len(acceptableSpaces) == 0:
            self.gameOver = True

    def GetAIMark(self, ai):
        if ai == 1:
            return "X"
        else:
            return "O"

    def CheckGameWon(self, boardSpaces, ai):
        #local variables
        aiSpaces = []
        
        # check which spaces have been occupied by which player
        for i in range (len(boardSpaces)):
            if boardSpaces[i] == self.GetAIMark(ai):
                aiSpaces.append(str(i + 1))

        if len(aiSpaces) > 2 and self.FoundWinningCombination(aiSpaces):
            self.gameOver = True
            if ai == 1:
                self.winner = "AI1"
            else:
                self.winner = "AI2"

    # check for winning combinations
    def FoundWinningCombination(self, aiSpaces):
        
        if self.CheckCombination(aiSpaces, ["1", "2", "3"]):
                return True
        elif self.CheckCombination(aiSpaces, ["4", "5", "6"]):
                return True
        elif self.CheckCombination(aiSpaces, ["7", "8", "9"]):
                return True
        elif self.CheckCombination(aiSpaces, ["1", "4", "7"]):
                return True
        elif self.CheckCombination(aiSpaces, ["2", "5", "8"]):
                return True
        elif self.CheckCombination(aiSpaces, ["3", "6", "9"]):
                return True
        elif self.CheckCombination(aiSpaces, ["1", "5", "9"]):
                return True
        elif self.CheckCombination(aiSpaces, ["3", "5", "7"]):
                return True
        else:
            return False

    # check a specified winning combination
    def CheckCombination(self, aiSpaces, set):
        for i in range(len(set)):
            if (set[i] not in aiSpaces):
                return False

        return True

    # render game state to the terminal
    def Render(self, ai, boardSpaces):
        '''
            Display the space numbers on the game board to the screen:
             1 | 2 | 2
            -----------
             4 | 5 | 6
            -----------
             7 | 8 | 9
        '''
        
        # display the results of the AI's move
        print("AI no. " + str(ai) + " placed an " + self.GetAIMark(ai) + " in space number " + str(self.input))

        # Display the current game board to screen
        print('    %s | %s | %s' % tuple(boardSpaces[:3]))
        print('   -----------')
        print('    %s | %s | %s' % tuple(boardSpaces[3:6]))
        print('   -----------')
        print('    %s | %s | %s' % tuple(boardSpaces[6:]))

        # Check if the game is over
        if self.gameOver:
            print("Game Over!")
            if self.winner != None:
                print("The winner is " + self.winner)
        else:
            input("Press enter to continue")
