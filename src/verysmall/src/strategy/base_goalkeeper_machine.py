from statemachine import StateMachine, State

class GoalKeeperMachine(StateMachine):
    """
    [State machine that creates a basic strategy for the goalkeeper]
    
    Extends:
        StateMachine
    
    Legend:
        (*) = State to indicate that

    States:
        stop                {[State]} -- [Robberies stopped in this state]
        freeball            {[State]} -- [Freeball state]
        normal              {[State]} -- [Normal Game state]
        penalt              {[State]} -- [Penalty state]
        meta                {[State]} -- [Meta state]
        keeper_defense      {[State]} -- [Base state of goalkeeper]
        keeper_forward      {[State]} -- [(*) the ball is in front of the goalkeeper]
        keeper_behind       {[State]} -- [(*) the ball is behind the goalkeeper]
        keeper_shoot        {[State]} -- [(*) the ball may be kicked by the goalkeeper]
        keeper_follow       {[State]} -- [(*) the ball has to be followed by the goalkeeper]
        keeper_line_ball    {[State]} -- [(*) Ball is on the line go goal]
        keeper_goal         {[State]} -- [(*) Opponent's goal]
    
    Transitions: 
        stop                {[Transition]} -- [Status of all stopped robberies]
        game_freeball       {[Transition]} -- [Transition that starts to freeball]
        game_normal         {[Transition]} -- [Game transition that occurs normally]
        game_penalt         {[Transition]} -- [Game transition stop to penalty]
        freeball_to_normal  {[Transition]} -- [Transition freeball to normal]
        penalt_to_normal    {[Transition]} -- [Transition penalty to normal]
        meta_to_normal      {[Transition]} -- [Transition meta to normal]

        normal_to_defense     {[Transition]} -- [Goalkeeper starting state]
        defense_to_forward    {[Transition]} -- [if the ball is in front of goalkeeper]
        defense_to_behind     {[Transition]} -- [if the ball is behind the goalkeeper]
        forward_to_shoot      {[Transition]} -- [that the goalkeeper can kick the ball]
        forward_to_follow     {[Transition]} -- [that the goalkeeper can not kick the ball]
        shoot_to_defense      {[Transition]} -- [goalkeeper kicked the ball out of the area]
        follow_to_defense     {[Transition]} -- [Looping following the ball to kick the ball]
        behind_to_line_ball   {[Transition]} -- [ball is on the goal line]
        line_ball_to_defense  {[Transition]} -- [Goalkeeper removed the ball from the line]
        line_ball_to_goal     {[Transition]} -- [Goalkeeper could not take the ball away]
        behind_to_goal        {[Transition]} -- [The goal scored]
    """

    # Base States
    stop      = State('Stop', initial=True)
    normal    = State('Normal')
    freeball  = State('FreeBall')
    penalty   = State('Penalty')
    meta      = State('Meta')

    ###### States of goalkeeper #####
    # State for when the ball is of defense
    # State of return of other states
    keeper_defense   = State('KeeperDefense')
    # State for when the ball is goalkeeper forward
    keeper_forward  = State('KeeperForward')
    # State for when the ball is goalkeeper behind
    keeper_behind   = State('KeeperBehind')

    # States for when ball is forward of goalkeeper
    keeper_shoot    = State('KeeperShoot')
    keeper_follow   = State('KeeperFollow')

    # States for when ball is forward of goalkeeper
    keeper_line_ball= State('KeeperLineBall')
    keeper_goal     = State('KeeperGoal')


    ######### Transitions basics states #########

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

    ######### Transitions of the goalkeeper #########

    ###  Dictionary
    # (*) = Transition to indicate

    # Goalkeeper starting state
    normal_to_defense  = normal.to(keeper_defense)
    # (*) if the ball is in front of goalkeeper
    defense_to_forward = keeper_defense.to(keeper_forward)
    # (*) if the ball is behind the goalkeeper
    defense_to_behind  = keeper_defense.to(keeper_behind)

    ##### Ball forward of the goalkeeper

    # (*) that the goalkeeper can kick the ball
    forward_to_shoot   = keeper_forward.to(keeper_shoot)
    # (*) that the goalkeeper can not kick the ball
    forward_to_follow  = keeper_forward.to(keeper_follow)

    # (*) goalkeeper kicked the ball out of the area
    shoot_to_defense   = keeper_shoot.to(keeper_defense)
    # (*) Looping following the ball to kick the ball
    follow_to_defense  = keeper_defense.to(keeper_defense)

    ##### Ball behind of the goalkeeper

    # (*) ball is on the goal line
    behind_to_line_ball = keeper_behind.to(keeper_line_ball)
    # (*) Goalkeeper removed the ball from the line
    line_ball_to_defense= keeper_line_ball.to(keeper_defense)
    # (*) Goalkeeper could not take the ball away
    line_ball_to_goal   = keeper_line_ball.to(keeper_goal)
    # (*) The goal scored
    behind_to_goal      = keeper_behind.to(keeper_goal)



class MyModel(object):
    def __init__(self, state):
        self.state = state