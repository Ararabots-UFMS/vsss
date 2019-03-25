import numpy as np
from new_seeker import *
seeker = NewSeeker(3,None)
a = np.zeros((4,4))
a[0][1] = 1
#a[1][3] = 1
a[2][3] = 1
b = []
visited = []
b = seeker.get_unions(a)
print(b)
obj_positions = [Vec2(0.7, 0.1), Vec2(2, 3) , Vec2(1,3)]
seeker.trackers[1].update(Vec2(1,3))
trackers_by_segment = [2,1]
for sucker in seeker.sort_by_distance_matrix(trackers_by_segment, obj_positions):
    print(sucker.to_list())