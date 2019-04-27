import numpy as np

from vision_module.seekers.new_seeker import *

seeker = NewSeeker(3,None)
a = np.zeros((4,4))
a[0][0] = 1
a[1][1] = 1
a[2][2] = 1
a[3][3] = 1
a[2][3] = 1

print(a)

b = []
b = seeker.get_unions(a)
print(b)