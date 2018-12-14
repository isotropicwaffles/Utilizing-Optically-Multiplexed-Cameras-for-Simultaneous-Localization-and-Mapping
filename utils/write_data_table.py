from os.path import isfile, join
from os import listdir

def write_file_table(file_to_write, path_to_time_stamps, image_folder):
    image_filenames = [f for f in listdir(image_folder) if isfile(join(image_folder, f))]
    
    with open(file_to_write, "w+") as table:
        table.write("#timestamp [ns],filename\n")
        
        with open(path_to_time_stamps, "r") as time_stamps:
            lines = time_stamps.readlines()
            i = 0

            for nanos in lines:
                table.write(nanos[:-1] + "," + image_filenames[i] + "\n")
                i += 1
    print("Wrote to " + file_to_write)

write_file_table("data4.csv", "MHMulti01.txt", "../external/ORB2_SLAM_Windows/Examples/Stereo/mav0/multiplex_output")
