from statemachine import StateMachine, State


class NaiveGK(StateMachine):

	    """
    Class for creation of the state machine of robberies,
    consisting of basic states of each robbery

    Extends:
        StateMachine

    Variables:
        stop                {[State]} -- [Robberies stopped in this state]
        freeball            {[State]} -- [Freeball state]
        normal              {[State]} -- [Normal Game state]
        penalt              {[State]} -- [Penalty state]
        meta                {[State]} -- [Meta state]
        stop                {[Transition]} -- [Status of all stopped robberies]
        game_freeball       {[Transition]} -- [Transition that starts to freeball]
        game_normal         {[Transition]} -- [Game transition that occurs normally]
        game_penalt         {[Transition]} -- [Game transition stop to penalty]
        freeball_to_normal  {[Transition]} -- [Transition freeball to normal]
        penalt_to_normal    {[Transition]} -- [Transition penalty to normal]
        meta_to_normal      {[Transition]} -- [Transition meta to normal]
    """
    # Base States
    stop      = State('Stop', initial=True)
    normal    = State('Normal')
    freeball  = State('FreeBall')
    penalty   = State('Penalty')
    meta      = State('Meta')

   	defend_ball		=	State('DEFEND')
	push_ball		=	State('PUSH_BALL')
	back_to_goal	=	State('BACK_TO_GOAL')

	track_ball		=	State('TRACK_BALL')
	right_most_goal	=	State('RMG')
	left_most_goal	=	State('LMG')
	follow_ball		=	State('FOLLOW_BALL')



    # Stop to freeball game
    stop_to_freeball = stop.to(freeball)
    # Stop to normal game
    stop_to_normal = stop.to(normal)
    # Stop to penalty
    stop_to_penalty = stop.to(penalty)
    # Stop to Meta
    stop_to_meta = stop.to(meta)
    # Freeball to normal game
    freeball_to_normal = freeball.to(normal)
    # Penalty to normal game
    penalty_to_normal = penalty.to(normal)
    # Meta to normal game
    meta_to_normal = meta.to(normal)

    go = stop_to_freeball | stop_to_normal | stop_to_penalty | freeball_to_normal | penalty_to_normal



	normal_to_defend_ball		= normal.to(defend)
	defend_ball_to_push_ball	= defend_ball.to(push_ball)
	push_ball_to_back_to_goal	= push_ball.to(back_to_goal)

	normal_to_track_ball			= normal.to(track_ball)
	track_ball_to_right_most_goal	= track_ball.to(right_most_goal) 
	track_ball_to_left_most_goal	= track_ball.to(left_most_goal)
	track_ball_to_follow_ball		= track_ball.to(follow_ball)

	right_most_goal_to_normal	= right_most_goal.to(normal)
	left_most_goal_to_normal	= left_most_goal.to(normal)
	follow_ball_to_normal		= follow_ball.to(normal)
	back_to_goal_to_normal		= back_to_goal.to(normal)


class MyModel(object):
    def __init__(self, state):
        self.state = state
