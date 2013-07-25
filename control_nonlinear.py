lambd = 0.9
kappa = 5
rho = 0.2

MORSE = True

##import numeric as np
import numpy as np
import math
import rospy
from std_msgs.msg import Float64 # TODO: switch to float32 since this is what nengo uses
from geometry_msgs.msg import Vector3, Wrench # TODO: make this a message specifically for position and velocity
from nav_msgs.msg import Odometry

##import nef
#from nengo import nef_theano as nef
import nengo_theano as nef
"""
class NonlinearControl(nef.Node):
    def __init__(self, name, lambd=lambd, kappa=kappa, rho=rho, D=1):
        nef.Node.__init__(self, name)
        
        self.lambd = lambd
        self.kappa = kappa
        self.rho = rho
        self.x = self.make_input('x', dimensions=D)
        self.dx = self.make_input('dx', dimensions=D)
        self.ddx = self.make_input('ddx', dimensions=D)
        self.x_desired = self.make_input('desired', dimensions=D)

        self.a = np.array([0.0, 0.0, 0.0])
        
        self.u = self.make_output('u', dimensions=D)
        self.s = self.make_output('s', dimensions=D)
        self.a_val = self.make_output('a', dimensions=3)
        
    def tick(self):        
        s = np.array(self.dx.get()) + self.lambd*(np.array(self.x.get()) - np.array(self.x_desired.get()))
        
        # shuld be ddx_r, which in this case is -lambda*dx
        Y = np.array([self.ddx.get()[0], self.dx.get()[0]*abs(self.dx.get()[0]), math.sin(self.x.get()[0])])
        # ddx_r = -lambd * dx
        self.u.set(sum(self.a * Y) - self.kappa * s)
        
        self.s.set(s)
        self.a_val.set(self.a)
    
        dt = 0.001
        self.a -= self.rho*(s*Y)*dt
    

class Physics(nef.Node):
    def __init__(self, name):
        nef.Node.__init__(self, name)
        
        
        self.x = self.make_output('x', dimensions=1)
        self.dx = self.make_output('dx', dimensions=1)
        self.ddx = self.make_output('ddx', dimensions=1)
        
        self.u = self.make_input('u', dimensions=1)
        
    def tick(self):
        dt = 0.001
        x = self.x._value[0]
        dx = self.dx._value[0]
        u = np.array(self.u.get())[0]

        J = 0.1
        b = 0
        mgl = 0.1

        ddx = u/J + dx*abs(dx)*b/J + mgl*math.sin(x)/J
        dx += ddx*dt
        x += dx*dt    
        
        while x>math.pi:
            x -= math.pi*2
        while x<-math.pi:
            x += math.pi*2
        
        
        #""
        if x>math.pi: 
            x = math.pi
            dx = 0
            ddx = 0
        if x<-math.pi: 
            x = -math.pi
            dx = 0
            ddx = 0
            
        
        if dx<-2: dx = -2
        if dx>2: dx = 2
        
        if ddx<-2: ddx = -2
        if ddx>2: ddx = 2
        #""
        
        self.x.set([x])
        self.dx.set([dx])
        self.ddx.set([ddx])
"""        
        
net = nef.Network('Nonlinear Control', seed=1)

##plant = net.add(Physics('plant'))

x = net.make_input('x',values=[0])
dx = net.make_input('dx',values=[0])
ddx = net.make_input('ddx',values=[0])

#control = net.add(NonlinearControl('control'))
##net.connect(plant.getOrigin('x'), control.getTermination('x'))
##net.connect(plant.getOrigin('dx'), control.getTermination('dx'))
##net.connect(plant.getOrigin('ddx'), control.getTermination('ddx'))

target = net.make_input('target', [1])
#net.connect('target', control.getTermination('desired'))


state = net.make('state', 300, 3, radius=2)
##net.connect(plant.getOrigin('x'), 'state', index_post=0)
##net.connect(plant.getOrigin('dx'), 'state', index_post=1)
##net.connect(plant.getOrigin('ddx'), 'state', index_post=2)
net.connect('x', 'state', index_post=0)
net.connect('dx', 'state', index_post=1)
net.connect('ddx', 'state', index_post=2)

s = net.make('s', 100, 1)
##net.connect('state', 's', transform=[lambd, 1, 0]) #TODO: figure out what the bug is
net.connect('state', 's')
net.connect('target', 's', weight=-lambd)

u = net.make('u', 100, 1, radius=1)
net.connect('s', 'u', weight=-kappa)

def learn(x):
    return [0]

net.connect('state', 'u', func=learn)
"""
class Learn(nef.Node):
    def __init__(self, name, origin):
        nef.Node.__init__(self, name)
        self.s = self.make_input('s', dimensions=1, pstc=0.01)
        self.Y = self.make_input('Y', dimensions=300, pstc=0.01)
        self.origin = origin
        self.counter = 0
    def tick(self):
        self.counter += 1
        if self.counter%10 == 0:
            delta = -rho * np.array(self.s.get())*0.00001
            Y = np.array(list(self.Y.get()))
            Y.shape = 300,1
            da = np.dot(Y, delta)
            decoder = np.array(self.origin.decoders)
            self.origin.decoders = decoder + da
"""        
def dummy():
  pass

class Learn(nef.SimpleNode):
    def __init__(self, name, origin):
        nef.SimpleNode.__init__(self, name)
        #self.s = self.add_input('s', dimensions=1, pstc=0.01)
        #self.Y = self.add_input('Y', dimensions=300, pstc=0.01)
        self.s = self.add_input('s', dimensions=1 )
        self.Y = self.add_input('Y', dimensions=300 )
        #self.origin = origin
        self.origin = { origin : net.get_origin( origin ) }
        #self.origin.func = learn # FIXME: is this right??
        #net.get_origin( origin ).func = learn # FIXME: is this right??
        net.get_origin( origin ).func = dummy # FIXME: is this right??
        self.counter = 0
    def tick(self):
        self.counter += 1
        if self.counter%10 == 0:
            delta = -rho * np.array(self.s.get())*0.00001
            Y = np.array(list(self.Y.get()))
            Y.shape = 300,1
            da = np.dot(Y, delta)
            decoder = np.array(self.origin.decoders)
            self.origin.decoders = decoder + da
        
#learn=net.add(Learn('learn', net.get('state').getOrigin('learn')))
#learn=net.add(Learn('learn', net.get_origin('state').getOrigin('learn')))
#learn=net.add(Learn('learn', net.get_origin('state')))
##net.connect('s', learn.getTermination('s'))
##net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y'))  

#learn = net.add( Learn( 'learn', net.get_origin('state') ) )
learn = net.add( Learn( 'learn', 'state' ) )
net.connect('s', 'learn:s')
#net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y'))  
net.connect('state', 'learn:Y')  


#net.connect(control.getOrigin('u'), plant.getTermination('u'))
#net.connect('u', plant.getTermination('u'))


        
def motion_callback( motion_msg ):
  x.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.x ] ) ) 
  dx.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.y ] ) ) 
  ddx.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.z ] ) ) 

def odom_callback( motion_msg ):
  x.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.pose.pose.orientation.x ] ) ) 
  dx.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.twist.twist.angular.x ] ) ) 
  ddx.origin['X'].decoded_output.set_value( np.float32( [ 0 ] ) ) # FIXME: get acceleration here

def main():
  if MORSE == True:
    rospy.init_node('pendulum_brain', anonymous=True)
    pub = rospy.Publisher('pendulum/control', Wrench)
    r = rospy.Rate(10) # 10 Hz
    # TODO: make this position and velocity
    sub = rospy.Subscriber('pendulum/motion', Odometry, odom_callback)
    while not rospy.is_shutdown():
      torque = u.origin['X'].decoded_output.get_value()[0]
      torque_msg = Wrench()
      torque_msg.torque.x = torque * 42
      pub.publish( torque_msg )
      net.run(0.01) # TODO: step the right amount of time
      r.sleep()
  else: #Gazebo
    rospy.init_node('pendulum_brain', anonymous=True)
    pub = rospy.Publisher('pendulum/torque', Float64)
    r = rospy.Rate(10) # 10 Hz
    # TODO: make this position and velocity
    sub = rospy.Subscriber('pendulum/motion', Vector3, motion_callback)
    while not rospy.is_shutdown():
      torque = u.origin['X'].decoded_output.get_value()[0]
      torque_msg = Float64()
      torque_msg.data = torque * 2337
      pub.publish( torque_msg )
      net.run(0.01) # TODO: step the right amount of time
      r.sleep()

if __name__ == '__main__':
  main()
#net.view()
#net.add_to_nengo()        
        
