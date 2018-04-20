import PID
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

pid = PID.PID(1.2, 1, 0.001, 1000.0, 1000.0, 0.0)

# number of interations
END = 50

feedback = 0

# experiment lists
feedback_list = []
time_list = []
setpoint_list = []

for i in range(1, END):
    output = pid.update(feedback)

    # wait for the impulse and upgrade the feedback using the pid output
    if pid.target != 0:
        # print feedback, output
        feedback += output

    # impulse signal on pid.target
    if i>1:
        pid.target = 1

    # time sample 60hz 
    time.sleep(0.016)

    # update experiment lists
    feedback_list.append(feedback)
    setpoint_list.append(pid.target)
    time_list.append(i)

# create list representing experiment time 
time_sm = np.array(time_list)
time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
feedback_smooth = spline(time_list, feedback_list, time_smooth)

# Plotting the impulse and response
plt.plot(time_smooth, feedback_smooth)
plt.plot(time_list, setpoint_list)
plt.xlim((0, END))
plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
plt.xlabel('time (s)')
plt.ylabel('PID (PV)')
plt.title('TEST PID')

plt.ylim((0, 2))

plt.grid(True)
plt.show()