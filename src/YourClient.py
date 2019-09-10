from src.World import *

from src.lib.decision import decision


def get_action(world):

    action = decision(world)

    return action
