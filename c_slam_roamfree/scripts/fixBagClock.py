import rosbag
import rospy
import sys

if len(sys.argv) < 3:
	print 'usage: fixBagClock.py <input_bag> <output_bag>'

last_t = None

with rosbag.Bag(sys.argv[2], 'w') as outbag:
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():

    	if topic == "/clock":
    		if last_t == None or msg.clock > last_t:
    			last_t = msg.clock
    			outbag.write(topic, msg, last_t)
    			
    	else:
    		outbag.write(topic, msg, last_t)
