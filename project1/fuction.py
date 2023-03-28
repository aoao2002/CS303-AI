import numpy as np
import time

a = [((0, 5), 52), ((1, 5), 47), ((2, 5), 91), ((2, 6), 47), ((3, 6), 47), ((4, 6), 57), ((5, 1), 69), ((5, 5), 49),
     ((5, 6), 49), ((6, 2), 71), ((6, 5), 95), ((7, 5), 52)]
t = time.time()
candidate_list = [node[0] for node in sorted(a, key=lambda x: x[1], reverse=True)]
print(time.time()-t)
print(candidate_list)
# print('我们的数组是：')
# print(a)
# print('\n')
# print('按列排序：')
# print(np.sort(a, axis=1))
# print('\n')
