from statemachine import StateMachine, State

class RobotStateMachine(StateMachine):
	"""
	Class for creation of the state machine of robberies, 
	consisting of basic states of each robbery
	
	Extends:
		StateMachine
	
	Variables:
		state_stop 			{[State]} -- [Robberies stopped in this state]
		state_freeball		{[State]} -- [Free ball state]
		state_normal 		{[State]} -- [Game playing normally]
		state_penalt 		{[State]} -- [Penalty status]
		stop 				{[Transition]} -- [Status of all stopped robberies]
		game_freeball 		{[Transition]} -- [Transition that starts to freeball]
		game_normal 		{[Transition]} -- [Game transition that occurs normally]
		game_penalt 		{[Transition]} -- [Game transition stop to penalty]
		freeball_to_normal 	{[Transition]} -- [Transition freeball to normal]
		penalt_to_normal 	{[Transition]} -- [Transition penalty to normal]
	"""

	# States basics
	state_stop 		= State('State_Stop', initial=True, value=1)
	state_normal 	= State('Normal', value=2)
	state_freeball 	= State('FreeBall', value=3)
	state_penalt 	= State('Penalt', value=4)

	# Transitions basics

	# Stop to freebal game
	game_freeball = state_stop.to(state_freeball)
	# Stop to normal game
	game_normal = state_stop.to(state_normal)
	# Stop to penalty
	game_penalt = state_stop.to(state_penalt)
	# Freball to normal game
	freeball_to_normal = state_freeball.to(state_normal)
	# Penalty to normal game
	penalt_to_normal = state_penalt.to(state_normal)

	# All transitions to stop game
	# stop = state_normal.to(state_stop) | state_freeball.to(state_stop) | state_penalt.to(state_stop) | state_stop.to(state_stop)

	def play(self):
		# print state
		pass

	# Stop all robbery
	def on_stop(self):
		pass

	# Callback of transition of the Stop Game to freeball
	def on_game_freeball(self):
		pass

	# Callback of transition of the Stop Game to Normal
	def on_game_normal(self):
		pass

	# Callback of transition of the Stop Game to Penalt
	def on_game_penalt(self):
		pass

	# Callback of transition of the Freeball to Normal
	def on_freeball_to_normal(self):
		pass

	# Callback of transition of the Penalt to Normal
	def on_penalt_to_normal(self):
		pass