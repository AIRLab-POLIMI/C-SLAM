import rosbag

import sys
from sensor_msgs.msg import Imu

import random

if len(sys.argv) < 8:
	print 'usage: create_noisy_IMU <input_bag> <output_bag> <acc_std> <gyro_std> <acc_bias> <gyro_bias> <new_imu_topic>'

acc_std = float(sys.argv[3])
gyro_std = float(sys.argv[4])
acc_bias = float(sys.argv[5])
gyro_bias = float(sys.argv[6])
new_imu_topic = sys.argv[7]

print '{0} {1}'.format(acc_std, gyro_std)

with rosbag.Bag(sys.argv[2], 'w') as outbag:
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
		
		# write again the same message
		outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

		if topic == "/firefly/ground_truth/imu":
			msg.angular_velocity.x = msg.angular_velocity.x + random.gauss(0.0, gyro_std);
			msg.angular_velocity.y = msg.angular_velocity.y + random.gauss(0.0, gyro_std);
			msg.angular_velocity.z = msg.angular_velocity.z + random.gauss(0.0, gyro_std);

			msg.linear_acceleration.x = msg.linear_acceleration.x + random.gauss(0.0, acc_std);
			msg.linear_acceleration.y = msg.linear_acceleration.y + random.gauss(0.0, acc_std);
			msg.linear_acceleration.z = msg.linear_acceleration.z + random.gauss(0.0, acc_std);

			outbag.write(new_imu_topic, msg, msg.header.stamp)