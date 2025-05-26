import os
import subprocess

DOWNLOAD_SCRIPT = "download-scannet.py"

# Set your desired start and end scene indices (inclusive)
START_IDX = 0
END_IDX = 50

# Directory to save downloads
OUTPUT_DIR = "scans"

def format_scene_name(idx):
    return f"scene{idx:04d}_00"

def download_scene(subdir, scene_id):
    scene_path = os.path.join(subdir, scene_id)
    if os.path.isdir(scene_path):
        print(f"✅ Skipping {scene_id} (already exists)")
        return

    os.makedirs(subdir, exist_ok=True)

    print(f"⬇️  Downloading {scene_id} .sens file...")
    subprocess.run([
        "python", DOWNLOAD_SCRIPT,
        "-o", subdir,
        "--id", scene_id,
        "--type", ".sens"
    ], input="\n\n", text=True)

    print(f"⬇️  Downloading {scene_id} task data...")
    subprocess.run([
        "python", DOWNLOAD_SCRIPT,
        "-o", subdir,
        "--id", scene_id,
        "--task_data"
    ], input="\n\n", text=True)

if __name__ == "__main__":
    for idx in range(START_IDX, END_IDX + 1):
        scene_id = format_scene_name(idx)
        download_scene(OUTPUT_DIR, scene_id)

    print("\n✅ Done downloading all selected scenes.")

