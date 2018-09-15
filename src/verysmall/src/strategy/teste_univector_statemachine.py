from univector_statemachine import AttackerWithUnivector
from robot_state_machine import  RobotStateMachine

model = AttackerWithUnivector()
model.statemachine.stop_to_normal()
model.statemachine.normal_game()
print model.statemachine.current_state
model.statemachine.current_state = model.statemachine.stop
print model.statemachine.current_state
