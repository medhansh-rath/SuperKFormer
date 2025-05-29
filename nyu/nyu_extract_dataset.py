import h5py
import numpy as np
import imageio
import os

file = h5py.File('nyu_depth_v2_labeled.mat', 'r')

images_ds = file['images']  # shape: (1449, 3, 640, 480)
depths_ds = file['depths']  # shape: (1449, 640, 480)
labels_ds = file['labels']  # shape: (1449, 1)
names_ds = file['names']  # shape: (894,)

nyu_dir = 'nyu_depth_v2_labeled'
if not os.path.exists(nyu_dir):
    os.makedirs(nyu_dir)

output_rgb_dir = os.path.join(nyu_dir, 'color')
output_depth_dir = os.path.join(nyu_dir, 'depth')
output_labels_dir = os.path.join(nyu_dir, 'labels')

os.makedirs(output_rgb_dir, exist_ok=True)
os.makedirs(output_depth_dir, exist_ok=True)
os.makedirs(output_labels_dir, exist_ok=True)

# # save the names of the labels
# with open(os.path.join(nyu_dir, 'names.txt'), 'w') as names_file:
#     for row in names_ds:
#         # row is likely an array of ASCII codes
#         name_str = ''.join([chr(c) for c in row if c != 0])
#         names_file.write(name_str + '\n')

names_path = os.path.join(nyu_dir, 'names.txt')
names_to_ids_path = os.path.join(nyu_dir, 'names_to_ids.txt')
class_names = []

# ...existing code...

with open(names_path, 'w') as names_file, open(names_to_ids_path, 'w') as map_file:
    for i in range(names_ds.shape[1]):
        ref = names_ds[0][i]
        name_dataset = file[ref]
        unicode_array = name_dataset[:].flatten()  # Read and flatten the UTF-16 chars
        # Convert UTF-16 code units to a string
        name = ''.join(chr(c) for c in unicode_array)
        names_file.write(name + '\n')
        map_file.write(f"{name}:{i}\n")

# ...existing code...

num_images = images_ds.shape[0]  # 1449

for i in range(num_images):
    rgb = images_ds[i, :, :, :]   # shape (3, 640, 480)
    depth = depths_ds[i, :, :]    # shape (640, 480)
    label = labels_ds[i, :, :]        # shape (1,)

    # Rearrange axes to (height, width, channels)
    rgb = np.transpose(rgb, (2, 1, 0))    # now (480, 640, 3)
    depth = np.transpose(depth, (1, 0))   # now (480, 640)
    label = np.transpose(label, (1, 0))   # now (480, 640)

    rgb = np.array(rgb, dtype=np.uint8)
    depth_mm = (depth * 1000).astype(np.uint16)

    rgb_filename = os.path.join(output_rgb_dir, f'{i:05d}.png')
    depth_filename = os.path.join(output_depth_dir, f'{i:05d}.png')
    label_filename = os.path.join(output_labels_dir, f'{i:05d}.png')

    imageio.imwrite(rgb_filename, rgb)

    imageio.imwrite(depth_filename, depth_mm)

    imageio.imwrite(label_filename, label.astype(np.uint8))

    if i % 100 == 0:
        print(f'Saved {i}/{num_images} images')

file.close()
print("✅ All images and depths saved!")

