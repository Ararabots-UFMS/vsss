from statemachine import StateMachine, State
import sys
import os
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/robot/"
from movement.functions.movement import Movement

movement = Movement([40, 0, 0], 10)

class AttackerWithUnivector(StateMachine):
    """Simple attacker with univector field"""

    # Robot States
    univector = State('Univector', initial=True)

    action = univector.to(univector)

    def on_action(self, robotPosition, robotVector, obstaclesPosition, obstaclesSpeed,  ballPosition):
        return movement.doUnivector(200, robotPosition, robotVector, obstaclesPosition, [[0,0]]*len(obstaclesPosition), ballPosition)