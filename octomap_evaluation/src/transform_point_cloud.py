import pypcd
import numpy as np

# read file
pc = pypcd.PointCloud.from_path('test.pcd')

# Transform matrix
A = [[1, 0, 0, 2],
     [0, 1, 0, 0],
     [0, 0, 1, 0],
     [0, 0, 0, 1]]

for f in pc.pc_data[['x', 'y', 'z']]:
    

print(pc.pc_data[['x', 'y', 'z']])

# save as binary compressed
pc.save_pcd('foo.pcd', compression='binary_compressed')
