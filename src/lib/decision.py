from src.World import World
from src.lib.predictor import predict_move


def decision(wm):
    action = predict_move(wm)

    return action
