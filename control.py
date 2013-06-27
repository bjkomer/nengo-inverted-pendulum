import numpy as np
from nengo import nef_theano as nef
import rospy
from std_msgs.msg import Float64 # TODO: switch to float32 since this is what nengo uses
from geometry_msgs.msg import Vector3 # TODO: make this a message specifically for position and velocity

# set up the nengo network
net=nef.Network('Pendulum')
#net.add_to_nengo()
# create the input connections, coming from the simulator
position_input = net.make_input('Position',value=[0])
velocity_input = net.make_input('Velocity',value=[0])

torque_population = net.make('Torque', neurons=30, dimensions=1) # population for the output torque

net.connect('Position', 'Torque', weight=-1.0)
net.connect('Velocity', 'Torque', weight=0.1)

def motion_callback( motion_msg ):
  position_input.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.x ] ) ) 
  velocity_input.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.y ] ) ) 

def main():
  rospy.init_node('pendulum_brain', anonymous=True)
  pub = rospy.Publisher('pendulum/torque', Float64)
  r = rospy.Rate(10) # 10 Hz
  # TODO: make this position and velocity
  sub = rospy.Subscriber('pendulum/motion', Vector3, motion_callback)
  while not rospy.is_shutdown():
    torque = torque_population.origin['X'].decoded_output.get_value()[0]
    torque_msg = Float64()
    torque_msg.data = torque * 2337
    pub.publish( torque_msg )
    net.run(0.01) # TODO: step the right amount of time
    r.sleep()

if __name__ == '__main__':
  main()
#net.view()
