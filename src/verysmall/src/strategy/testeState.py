from univector_statemachine import AttackerWithUnivector
from robot_state_machine import ModelMachine

a = ModelMachine(state='Stop')
tt = AttackerWithUnivector(a)