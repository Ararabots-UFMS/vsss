from threading import Lock
import numpy as np
from random import randint
import rospy
from time import time

from verysmall.msg import game_topic, things_position
from ROS.ros_game_topic_publisher import GameTopicPublisher
from strategy.behaviour import BlackBoard
from utils.model import Model
from strategy.arena_utils import LEFT, RIGHT
from utils.profiling_tools import log_fatal


class Trainer:
    PLAYERS = {"Attacker", "Defender", "Goalkeeper"}
    BEHAVIOURS = {}
    BEHAVIOURS_LUT = {}

    NORMAL_STATE = 1
    REFRESH_TIME = 2
    def __init__(self, owner_id: str = 'Player_One' ):
        self._roles = {}
        self._last_publish_time = time()
    
        # Load the database
        model = Model()
    
        self.load_roles_ids(model.robot_roles)

        self._game_topic = game_topic()
        self._blackboard = BlackBoard()
        self._roles = self._game_topic.robot_roles

        self._gametopic_publisher = GameTopicPublisher(False, model.game_opt, model.robot_params, 
                                                        model.robot_roles, owner_id = owner_id)

        self._game_topic_locker = Lock()
        self._blackboard_locker = Lock()

        rospy.Subscriber('things_position', things_position, self._things_position_topic_callback, queue_size=5)
        rospy.Subscriber('game_topic_'+owner_id, game_topic, self._game_topic_callback, queue_size=1)

    def load_roles_ids(self, roles: dict) -> None:
        Trainer.BEHAVIOURS = {player: roles[player] 
                              for player in Trainer.PLAYERS}
        Trainer.BEHAVIOURS_LUT = {v: k for k, v in Trainer.BEHAVIOURS.items()}
        log_fatal(Trainer.BEHAVIOURS_LUT)

    
    # subscriber
    def _game_topic_callback(self, topic: game_topic) -> None:
        self._game_topic_locker.acquire()
        self._game_topic = topic
        self._game_topic_locker.release()

        self._run_trainer()
    
    
    def _things_position_topic_callback(self, topic: things_position) -> None:
        self._blackboard_locker.acquire()

        self._blackboard.ball.position = np.array(topic.ball_pos) / 100.0

        if self._game_topic.team_color == 1:  # yellow
            friends_position = np.array(topic.yellow_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(topic.yellow_team_orientation) / 10000.0

            enemies_position = np.array(topic.blue_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(topic.blue_team_orientation) / 10000.0
        else:  # blue
            friends_position = np.array(topic.blue_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(topic.blue_team_orientation) / 10000.0

            enemies_position = np.array(topic.yellow_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(topic.yellow_team_orientation) / 10000.0

        
        self._blackboard.home_team.set_team_variables(friends_position,
                                                      friends_orientation)

        self._blackboard.enemy_team.set_team_variables(enemies_position,
                                                       enemies_orientation)
        
        self._blackboard_locker.release()

        self._run_trainer()
    

    def _run_trainer(self) -> None:
        self._game_topic_locker.acquire()

        dt = time() - self._last_publish_time
        if self._game_topic.game_state != Trainer.NORMAL_STATE or\
           dt < Trainer.REFRESH_TIME:
            self._game_topic_locker.release()
            return
        
        self._blackboard_locker.acquire()
        
        self._run_strategy()

        if self._roles == self._game_topic.robot_roles:
            self._game_topic_locker.release()
            self._blackboard_locker.release()
            return

        self._gametopic_publisher.set_message(self._game_topic.game_state,
                                              self._game_topic.team_side,
                                              self._game_topic.team_color,
                                              self._roles,
                                              self._game_topic.robot_tags,
                                              self._game_topic.penalty_robot,
                                              self._game_topic.freeball_robot,
                                              self._game_topic.meta_robot)
                                              
        self._gametopic_publisher.publish()
        self._last_publish_time = time()

        self._game_topic_locker.release()
        self._blackboard_locker.release()


    def _run_strategy(self) -> None:
        # ball in critical area check
        team_side = self._blackboard.home_goal.side
        ball = self._blackboard.ball.position
        if ball[0] < 30 and team_side == LEFT:
            return
        elif ball[0] > 120 and team_side == RIGHT:
            return

        team_pos = self._blackboard.home_team.positions

        active_tags = np.argwhere(np.any(team_pos, axis=1))[..., 0]
        rospy.logfatal(active_tags)
        
        if len(active_tags) != 3:
            return

        tags_robot_table = {self._game_topic.robot_tags[i]: i 
                            for i in range(len(self._game_topic.robot_tags))}
        active_robots = [tags_robot_table[t] for t in active_tags]

        # will help further
        lut = {i: active_robots[i] for i in range(len(active_robots))}

        robots_positions = team_pos[active_tags, ...]

        distances = np.linalg.norm(robots_positions - ball, axis=1)
        ids = np.argsort(distances, kind="stable")

        # attacker
        attacker_id = lut[ids[0]]

        # defender
        defender_id = lut[ids[1]]

        goalkeeper_id = lut[ids[2]]

        roles = list(self._roles)

        roles[attacker_id] = Trainer.BEHAVIOURS["Attacker"]
        roles[defender_id] = Trainer.BEHAVIOURS["Defender"]
        roles[goalkeeper_id] = Trainer.BEHAVIOURS["Goalkeeper"]

        self._roles = bytes(roles)