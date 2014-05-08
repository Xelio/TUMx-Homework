import quadrotor.command as cmd
from math import sqrt
from math import atan
from math import degrees

def plan_mission(mission):

    # this is an example illustrating the different motion commands,
    # replace them with your own commands and activate all beacons
    commands  = [
        cmd.up(1),
        cmd.turn_right(degrees(atan(2))),
        cmd.forward(sqrt(1 + 2**2)),
        cmd.turn_right(degrees(atan(0.5)) + 45),
        cmd.backward(sqrt(4**2 + 4**2)),
        cmd.turn_left(45),
        cmd.forward(4),
        cmd.turn_left(degrees(atan(0.5))),
        cmd.backward(sqrt(2**2 + 4**2)),
        cmd.turn_left(degrees(atan(2))),
        cmd.backward(2),
        cmd.turn_right(90),
        cmd.forward(2),
        cmd.turn_left(45),
        cmd.forward(sqrt(2**2 + 2**2)),
        cmd.down(1)
    ]

    mission.add_commands(commands)
