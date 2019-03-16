import TicTacToeGame as gameModule

while True:
    game = gameModule.TicTacToeGame()

    replay = game.GameLoop()

    if replay is not True:
        break