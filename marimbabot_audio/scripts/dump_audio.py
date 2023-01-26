#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioDataStamped
import sys

dump = None
msg_cnt = 0

def cb(msg):
  global msg_cnt
  msg_cnt += 1
  rospy.loginfo(len(msg.audio.data))
  dump.write(bytes(msg.audio.data))

def setup():
  global dump
  filename = sys.argv[1] if len(sys.argv) > 1 else "dump.pcm"
  dump = open(filename, 'wb')
  rospy.Subscriber('audio_stamped', AudioDataStamped, cb, queue_size=100, tcp_nodelay= True)
  rospy.Timer(rospy.Duration(1.0), status)
  rospy.loginfo('setup complete')

def status(event):
  rospy.loginfo('processed {0} messages so far'.format(msg_cnt))

def shutdown():
  if dump:
    dump.close()

if __name__ == '__main__':
   rospy.init_node('audio_dumper')
   setup()
   rospy.on_shutdown(shutdown)
   rospy.spin()
