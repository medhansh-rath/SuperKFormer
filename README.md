# SuperKFormer
Superpixel Transformer for RGBD Data that Uses K-Means

Download the [ScanNet](http://www.scan-net.org/) v2 dataset. Use 
'''
python ./download-scannet.py --preprocessed_frames -o {OUTPUT_DIR}
'''

Use the create_organized_point_cloud.py script to create .pcd files for VCCS from RGB and depth images
