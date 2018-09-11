from robotstatemachine import RobotStateMachine
from model_robot_state_machine import ModelRobotStateMachine

def main():
	# print "ok"
	estado = ModelRobotStateMachine(state='state_stop')
	trr = RobotStateMachine(estado)
	# trr.run('game_freeball')

	# state.state = 'state_penalt'

	print trr

	
	
	# print trr.current_state

if __name__ == '__main__':
	main()