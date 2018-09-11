from robotstatemachine import RobotStateMachine, ModelMachine

def main():
	# para chamar um objeto na classe deve chamar de estado a id do stado cado tenha
	estado = ModelMachine(state=1)
	trr = RobotStateMachine(estado)
	trr.run('game_freeball')

	# state.state = 'state_penalt'

	print estado.state
	print trr.current_state

	estado.stop()
	print trr.current_state
	
	# print trr.current_state

if __name__ == '__main__':
	main()