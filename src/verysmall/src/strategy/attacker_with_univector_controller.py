from attacker_with_univector import AttackerWithUnivector

class AttackerWithUnivectorController():

    def __init__(self):
        self.position = None
        self.orientation = None
        self.robot_speed = None
        self.enemies_position = None
        self.enemies_speed = None
        self.ball_position = None

        self.AttackerWithUnivector = AttackerWithUnivector()


    def update_game_information(self, position, orientation, robot_speed, enemies_position, enemies_speed, ball_position):
        """
        Update game variables
        :param position:
        :param orientation:
        :param robot_speed:
        :param enemies_position:
        :param enemies_speed:
        :param ball_position:
        """
        self.position = position
        self.orientation = orientation
        self.robot_speed = robot_speed
        self.enemies_position = enemies_position
        self.enemies_speed = enemies_speed
        self.ball_position = ball_position

    def set_to_stop_game(self):
        pass

    def in_normal_game(self):
        if self.AttackerWithUnivector.is_stop:
            self.AttackerWithUnivector.stop_to_normal()

        elif self.AttackerWithUnivector.is_normal:
            self.AttackerWithUnivector.normal_to_univector()

        elif self.AttackerWithUnivector.is_univector:
            self.AttackerWithUnivector.univector_to_univector()
