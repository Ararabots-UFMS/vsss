from univector_statemachine import AttackerWithUnivector
from robot_state_machine import  RobotStateMachine

model = AttackerWithUnivector()
model.statemachine.stop_to_normal()
print model.statemachine.current_state
model.statemachine.current_state = model.statemachine.stop
print model.statemachine.current_state
