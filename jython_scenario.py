from morse.builder import *

pendulum = Pendulum('Pendulum')
pendulum.translate(x=0.0, z=0)
pendulum.properties(Object = True, Graspable = False, Label = "ROBOT")

driver = ATRV()
driver.translate(x=2.0, y=2.0)

torque = ForceTorque()
torque.frequency(frequency=100)
odom = Odometry()
odom.level( "relative" )
odom.frequency(frequency=100)

pendulum.append( torque )
pendulum.append( odom )

semantic = SemanticCamera()
semantic.translate(x=0.2, y=0.3, z=0.9)
semantic.rotate(x=0.25, y=0.0, z=0.0)
semantic.frequency(frequency=10)
pendulum.append(semantic)

keyboard = Keyboard()
keyboard.properties(Speed=3.0)
driver.append(keyboard)

# FIXME: temporary for testing latency issues
#odom.frequency(frequency=60)

torque.add_interface( 'socket' )
odom.add_interface( 'socket' )

env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
