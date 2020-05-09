import datetime as dt
import glob
import os
import matplotlib.pyplot as plt
import numpy as np

data_path = "/media/wegatron/data/data/kitti/2011_09_26/2011_09_26_drive_0014_extract"
def load_timestamps(data='oxts'):
    """Load timestamps from file."""
    timestamp_file = os.path.join(
        data_path, data, 'timestamps.txt')

    # Read and parse the timestamps
    timestamps = []
    with open(timestamp_file, 'r') as f:
        for line in f.readlines():
            # NB: datetime only supports microseconds, but KITTI timestamps
            # give nanoseconds, so need to truncate last 4 characters to
            # get rid of \n (counts as 1) and extra 3 digits
            t = dt.datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            t = dt.datetime.timestamp(t)
            timestamps.append(t)

    # Subselect the chosen range of frames, if any
    return timestamps
timestamps = np.array(load_timestamps())
x = np.arange(0, len(timestamps))

last_timestamp = timestamps[:-1]
curr_timestamp = timestamps[1:]
dt = np.array(curr_timestamp - last_timestamp) #计算前后帧时间差

print("dt > 0.015: \n{}".format(dt[dt> 0.015])) # 打印前后帧时间差大于0.015的IMU index
dt = dt.tolist()
dt.append(0.01)
dt = np.array(dt)
print("dt > 0.015: \n{}".format(x[dt> 0.015])) # 打印时间差大于0.015的具体时间差
plt.plot(x, timestamps, 'r', label='imu')  # 可视化ＩＭＵ的时间戳
plt.show()
