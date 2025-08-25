import time
from gz.transport14 import Node
from gz.msgs.twist_pb2 import Twist

#Creating A node object (program that will make communication)
node = Node()
pub = node.advertise("/model/diffbot/cmd_vel", Twist)

#Creating and Populating A message object
msg = Twist()
msg.linear.x = 0.3

# Sending the message
for _ in range(50):
    pub.publish(msg)
    time.sleep(0.1)
