# SuperKFormer
Superpixel Transformer for RGBD Data that Uses K-Means

NYU v2:

Download the NYU v2 labelled dataset using:

```
wget http://horatio.cs.nyu.edu/mit/silberman/nyu_depth_v2/nyu_depth_v2_labeled.mat
```
To extract the dataset use:
```
python nyu/extract_dataset.py
```

To visualize the results use:
```
python nyu/visualize_point_cloud.py
```

ScanNet v2:
Download the [ScanNet](http://www.scan-net.org/) v2 dataset. Use 

```
python ./download-scannet.py --preprocessed_frames -o {OUTPUT_DIR}
```

To create .pcd files from RGB and depth images use:
```
python sannet/scannet_create_pcd.py
```
