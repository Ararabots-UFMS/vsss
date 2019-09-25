import rospy


class Logger:
    def __init__(self):
        self.last_fatal_log = ''
        self.last_warn_log = ''

    def log_fatal(self, log):
        if log != self.last_fatal_log:
            rospy.logfatal(log)
            self.last_fatal_log = log

    def log_warn(self, log):
        if log != self.last_warn_log:
            rospy.logwarn(log)
            self.last_warn_log = log


logger = Logger()
log_fatal = logger.log_fatal
log_warn = logger.log_warn
