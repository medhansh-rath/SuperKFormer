#!/bin/bash

NYU_DIR="$1"

if [ -z "$NYU_DIR" ]; then
  echo "Usage: $0 <NYU_dir>"
  exit 1
fi

  RGB_DIR="$NYU_DIR/color"
  DEPTH_DIR="$NYU_DIR/depth"
  OUTPUT_DIR="$NYU_DIR/supervoxel"

  if [ -d "$RGB_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    for rgb_file in "$RGB_DIR"/*.png; do
      image_base=$(basename "$rgb_file" .png)
      ./build/supervoxel_generator -r "$RGB_DIR/$image_base.png" -d "$DEPTH_DIR/$image_base.png" --NT \
        -O "${OUTPUT_DIR}/${image_base}_output.png" -L "${OUTPUT_DIR}/${image_base}_labels.png"
        echo "Processed $image_base, output saved to ${OUTPUT_DIR}/${image_base}_output.png and ${OUTPUT_DIR}/${image_base}_labels.png"
    done
    else
    echo "Warning: RGB directory not found for NYU dataset"
  fi