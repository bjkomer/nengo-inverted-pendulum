from morse.builder import *

pendulum = Pendulum('Pendulum')
pendulum.translate(x=0.0, z=0)

driver = ATRV()
driver.translate(x=-2.0, y=-2.0)

torque = ForceTorque()
odom = Odometry()
odom.level( "relative" )

pendulum.append( torque )
pendulum.append( odom )

keyboard = Keyboard()
keyboard.properties(Speed=3.0)
driver.append(keyboard)

torque.add_interface( 'socket' )
odom.add_interface( 'socket' )

env = Environment('outdoors')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
