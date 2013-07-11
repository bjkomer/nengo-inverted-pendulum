from morse.builder import *

pendulum = Pendulum('pendulum')
pendulum.translate(x=0.0, z=0)
pendulum.properties(Object = True, Graspable = False, Label = "ROBOT")

driver = ATRV()
driver.translate(x=2.0, y=2.0)

torque = ForceTorque()
odom = Odometry()
odom.level( "relative" )

pendulum.append( torque )
pendulum.append( odom )

semantic = SemanticCamera()
semantic.translate(x=0.2, y=0.3, z=0.9)
semantic.rotate(x=0.25, y=0.0, z=0.0)
semantic.frequency(frequency=30)
pendulum.append(semantic)

keyboard = Keyboard()
keyboard.properties(Speed=3.0)
driver.append(keyboard)

torque.add_interface( 'ros', topic='pendulum/control' )
odom.add_interface( 'ros', topic='pendulum/motion' )

env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(semantic)
