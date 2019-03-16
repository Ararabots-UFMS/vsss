import numpy as np
from new_seeker import NewSeeker
seeker = NewSeeker(3,None)
a = np.zeros((4,4))
a[0][1] = 1
#a[1][3] = 1
a[2][3] = 1
b = []
visited = []
b = seeker.get_unions(a)
print(b)
