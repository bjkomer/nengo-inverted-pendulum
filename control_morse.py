# Controller for the morse simulation ( different message formats than the Gazebo one currently )

import numpy as np
from nengo import nef_theano as nef
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench

# set up the nengo network
net=nef.Network('Pendulum')
#net.add_to_nengo()
# create the input connections, coming from the simulator
position_input = net.make_input('Position',value=[0])
velocity_input = net.make_input('Velocity',value=[0])

torque_population = net.make('Torque', neurons=600, dimensions=1) # population for the output torque

net.connect('Position', 'Torque', weight=-0.9)
net.connect('Velocity', 'Torque', weight=-0.3)

def motion_callback( motion_msg ):
  position_input.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.pose.pose.orientation.x ] ) ) 
  velocity_input.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.twist.twist.angular.x ] ) ) 

def main():
  rospy.init_node('pendulum_brain', anonymous=True)
  pub = rospy.Publisher('pendulum/control', Wrench)
  r = rospy.Rate(10) # 10 Hz
  sub = rospy.Subscriber('pendulum/motion', Odometry, motion_callback)
  while not rospy.is_shutdown():
    torque = torque_population.origin['X'].decoded_output.get_value()[0]
    torque_msg = Wrench()
    torque_msg.torque.x = torque * 42#* 2337
    pub.publish( torque_msg )
    net.run(0.01) # TODO: step the right amount of time
    r.sleep()

if __name__ == '__main__':
  main()
#net.view()
