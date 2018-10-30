from statemachine import StateMachine, State


class AdvancedGK(StateMachine):

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
    stop = State('stop', initial=True)
    normal    = State('Normal')
    freeball  = State('FreeBall')
    penalty   = State('Penalty')
    meta      = State('Meta')

    defend_ball         =   State('DEFEND')
    seek_ball           =   State('SEEK_BALL')
    goal                =   State('GOAL')
    out_of_area         =   State('OUT_OF_AREA')
    spin                =   State('SPIN')
    go_to_ball          =   State('GO_TO_BALL')




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

    normal_to_defend_ball       = normal.to(defend_ball)
    normal_to_seek_ball         = normal.to(seek_ball) 
    normal_to_goal              = normal.to(goal)
    normal_to_out_of_area       = normal.to(out_of_area)


    defend_ball_to_seek_ball        = defend_ball.to(seek_ball)
    defend_ball_to_goal             = defend_ball.to(goal)
    defend_ball_to_out_of_area      = defend_ball.to(out_of_area)
    defend_ball_to_spin             = defend_ball.to(spin)
    defend_ball_to_go_to_ball       = defend_ball.to(go_to_ball)

    seek_ball_to_defend_ball        = seek_ball.to(defend_ball)
    seek_ball_to_goal               = seek_ball.to(goal)
    seek_ball_to_out_of_area        = seek_ball.to(out_of_area)

    goal_to_defend_ball             = goal.to(defend_ball)
    goal_to_seek_ball               = goal.to(seek_ball)

    out_of_area_to_defend_ball      = out_of_area.to(defend_ball)
    out_of_area_to_seek_ball        = out_of_area.to(seek_ball)


    
        

class MyModel(object):
    def __init__(self, state):
        self.state = state
