#!/home/ou/software/anaconda3/envs/py2/bin/python
import numpy as np
from pathlib import Path
import os

split = 'training'
dataset_dir = "/home/ou/Documents/dataset/gazebo_pvrcnn_dataset/nice/my_kitti/object"
split_dir = "/home/ou/Documents/dataset/gazebo_pvrcnn_dataset/nice/my_kitti/object/ImageSets"

sample_id = []
for filename in os.listdir(os.path.join(dataset_dir, split, "velodyne")):
    sample_id.append(str(filename.split('.')[0]))
sample_id.sort()
indices = np.random.permutation(len(sample_id))
train_indices = indices[:int(len(indices) / 2)].tolist()
val_indices = indices[int(len(indices) / 2):].tolist()

os.system('rm -r %s' % split_dir)
Path(split_dir).mkdir(parents=True)

with open(os.path.join(split_dir, "train.txt"), 'w') as f:
    train_indices.sort()
    for sample in train_indices:
        f.write(sample_id[sample] + '\n')

with open(os.path.join(split_dir, "val.txt"), 'w') as f:
    val_indices.sort()
    for sample in val_indices:
        f.write(sample_id[sample] + '\n')

print train_indices, val_indices
