from os.path import isfile, join
from os import listdir, rename

def rename_frames(timestamps_file, image_folder):
    files = sorted([f for f in listdir(image_folder) if isfile(join(image_folder, f))])

    with open(timestamps_file, "r") as ts:
        ts_lines = ts.readlines()

        for i in range(len(ts_lines)):
            timestamp = ts_lines[i][:-1]
            filename = files[i]

            rename(image_folder + "/" + filename, image_folder + "/" + timestamp + ".png")
    print("Completed rename operation")

rename_frames("MHMulti01.txt", "../external/ORB2_SLAM_Windows/Examples/Stereo/mav0/multiplex_output")
