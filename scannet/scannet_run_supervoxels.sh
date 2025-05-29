#!/bin/bash

SCANNET_DIR="$1"

if [ -z "$SCANNET_DIR" ]; then
  echo "Usage: $0 <scannet_dir>"
  exit 1
fi

for SCENE in $(ls -1v "$SCANNET_DIR"); do
  SCENE_PATH="$SCANNET_DIR/$SCENE"
  PCD_DIR="$SCENE_PATH/pcd"
  OUTPUT_DIR="$SCENE_PATH/supervoxel"

  if [ -d "$PCD_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    for pcd_file in "$PCD_DIR"/*.pcd; do
      pcd_base=$(basename "$pcd_file" .pcd)
      ./build/supervoxel_generator -p "$pcd_file" --NT \
        -O "${OUTPUT_DIR}/${pcd_base}_output.png" -L "${OUTPUT_DIR}/${pcd_base}_labels.png"
    done
  else
    echo "Warning: PCD directory not found for scene $SCENE"
  fi
done
