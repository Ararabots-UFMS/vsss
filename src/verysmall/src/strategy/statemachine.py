from statemachine import StateMachine, State

class RobotStateMachine(StateMachine):
	"""
	Class for creation of the state machine of robberies, 
	consisting of basic states of each robbery
	
	Extends:
		StateMachine
	
	Variables:
		stop_game {[type]} -- [description]
		free_game {[type]} -- [description]
		normal_game {[type]} -- [description]
		penalt_game {[type]} -- [description]
		game_freeball {[type]} -- [description]
		game_normal {[type]} -- [description]
		game_penalt {[type]} -- [description]
		freeball_to_normal {[type]} -- [description]
		penalt_to_normal {[type]} -- [description]
	"""
	# States basics
	stop_game = State('Stop_Game', initial=True)
	free_game = State('FreeBall')
	normal_game = State('Normal')
	penalt_game = State('Penalt')

	# Transitions basics
	game_freeball = stop_game.to(free_game)
	game_normal = stop_game.to(normal_game)
	game_penalt = stop_game.to(penalt_game)
	freeball_to_normal = free_game.to(normal_game)
	penalt_to_normal = penalt_game.to(normal_game)

	# Callback of transition of the Stop Game to freeball
	def on_game_freball():
		pass

	# Callback of transition of the Stop Game to Normal
	def on_game_normal():
		pass

	# Callback of transition of the Stop Game to Penalt
	def on_game_penalt():
		pass

	# Callback of transition of the Freeball to Normal
	def on_freball_to_normal():
		pass

	# Callback of transition of the Penalt to Normal
	def on_penalt_to_normal():
		pass