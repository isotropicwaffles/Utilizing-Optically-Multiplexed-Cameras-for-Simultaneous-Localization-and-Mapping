import time

def write_timestamp_file(filename, fps, number_of_files):
    end_time = time_to_whole_nanos(time.time())
    start_time = end_time - time_to_whole_nanos(number_of_files * (1. / fps))

    print("Writing to " + filename)
    with open(filename, "w+") as f:
        for i in range(number_of_files):
            f.write(str(start_time + time_to_whole_nanos(i * (1. / fps))) + "\n")
    print(filename + " written")


def time_to_whole_nanos(time_in_seconds):
    return int(time_in_seconds * 1e9)

def time_to_str(nano_time):
    return "%.0f" % nano_time

write_timestamp_file("MHMulti01.txt", 30, 3682)
