import pyrealsense2 as rs

ctx = rs.context()
devs = ctx.query_devices() # devs是device_list类
device_num = devs.size()

# print(device_num)
# 1
# print(len(devs))
# 1

# dev = devs[0]
# print(type(dev))
# <class 'pyrealsense2.pyrealsense2.device'>

for dev in devs:
    serial_number = dev.get_info(rs.camera_info.serial_number)
    print(serial_number)



