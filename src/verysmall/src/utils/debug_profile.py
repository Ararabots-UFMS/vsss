import cProfile
import sys
import io
import rospy

class DebugProfile(cProfile.Profile):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.enabled = False
        self.current_logs = None

    def enable(self) -> None:
        self.enabled = True
        super().enable()
    
    def log(self) -> None:

        old_stdout = sys.stdout
        new_stdout = io.StringIO()
        sys.stdout = new_stdout

        self.print_stats()

        sys.stdout = old_stdout

        self.current_logs = new_stdout.getvalue()
        # TODO: fazer algo com isso!!!
