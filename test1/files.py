import os

# every file test1 reads or writes, found from this file's own folder
# so the scripts work no matter where they are run from
TEST_FOLDER = os.path.dirname(os.path.abspath(__file__))
DATA_FOLDER = os.path.join(TEST_FOLDER, "data")
RESULTS_FOLDER = os.path.join(TEST_FOLDER, "results")
PLOTS_FOLDER = os.path.join(TEST_FOLDER, "plots")

for folder in (DATA_FOLDER, RESULTS_FOLDER, PLOTS_FOLDER):
    os.makedirs(folder, exist_ok=True)

POINT_CLOUD_FILE = os.path.join(DATA_FOLDER, "point_cloud.npy")
SLAM_FILE = os.path.join(DATA_FOLDER, "slam_paths.npy")
MAP_FILE = os.path.join(DATA_FOLDER, "map.npy")
PATHS_FILE = os.path.join(DATA_FOLDER, "paths.json")
FLOWN_FILE = os.path.join(PLOTS_FOLDER, "flown_paths.json")
RESULTS_FILE = os.path.join(RESULTS_FOLDER, "test1_results.csv")
