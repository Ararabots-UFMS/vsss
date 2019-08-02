import subprocess as sb
from rospy import logfatal

class ProcessKiller():
	"""
	This class takes a list of process, and if they are running, kill them all
	"""
	def __init__(self, _array_of_process):
		"""
		:param process_name: Array
		"""
		self.array_of_process = _array_of_process
		self.kill_them_all()

	def get_list_of_pids(self, process_name):
		"""
		Return an array of process ids 
		:param process_name: Array
		:return: Array
		"""
		try:
			list_of_pid = sb.check_output("pgrep "+process_name, shell=True).split("\n")
			list_of_pid.pop()
		except Exception as e:
			list_of_pid = []
			
		return list_of_pid

	def kill_them_all(self):
		"""
		Kill all process in array of class
		:return: nothing
		"""
		for process_name in self.array_of_process:
			for pid in self.get_list_of_pids(process_name):
				sb.check_output("kill -9 " + pid, shell=True)


if __name__ == '__main__':
	ProcessKiller(["vision_node","robot"])