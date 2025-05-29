#!/bin/bash

INPUT_DIR="$1"

if [ -z "$INPUT_DIR" ]; then
  echo "Usage: $0 <input_folder>"
  exit 1
fi

for pcd_file in "$INPUT_DIR"/*.pcd; do
  base=$(basename "$pcd_file" .pcd)
  ./build/supervoxel_generator -p "$pcd_file" --NT \
    -O "${INPUT_DIR}/${base}_output.png" -L "${INPUT_DIR}/${base}_labels.png"
done