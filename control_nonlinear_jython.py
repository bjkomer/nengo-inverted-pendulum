lambd = 0.9
kappa = 5
rho = 0.2

import numeric as np
import math
import socket
import sys
sys.path.append('/home/komer/Downloads/jyson-1.0.2/src')
sys.path.append('/home/komer/Downloads/jyson-1.0.2/lib/jyson-1.0.2.jar')
import com.xhaus.jyson.JysonCodec as json # Jython version of json
from com.xhaus.jyson import JSONDecodeError, JSONEncodeError

HOST = '127.0.0.1'
PORT = 60000

import nef

def connect_port( port ):
  """ Establish the connection with the given MORSE port"""
  sock = None

  for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res
    try:
      sock = socket.socket(af, socktype, proto)
    except socket.error:
      sock = None
      continue
    try:
      sock.connect(sa)
    except socket.error:
      sock.close()
      sock = None
      continue
    break

  return sock

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
        
        # Connect with MORSE through a socket
        self.sock = connect_port( PORT )
        if not self.sock:
          sys.exit(1)

        self._id = 0
        print( "sock connected" )
        
        self.x = self.make_output('x', dimensions=1)
        self.dx = self.make_output('dx', dimensions=1)
        self.ddx = self.make_output('ddx', dimensions=1)
        
        self.u = self.make_input('u', dimensions=1)
        
    def tick(self):
        dt = 0.001
        x = self.x._value[0]
        dx = self.dx._value[0]
        u = np.array(self.u.get())[0]
        
        # Send u to the simulator as a torque

        J = 0.1
        b = 0
        mgl = 0.1

        ddx = u/J + dx*abs(dx)*b/J + mgl*math.sin(x)/J
        dx += ddx*dt
        x += dx*dt    
        
        # Read in the angular position and velocity from the simulator
        morse = self.sock.makefile("r")
        try:
          data_in = json.loads(morse.readline())
          x = data_in["roll"]
          dx = data_in["wx"]
        except JSONDecodeError:
          print "ERROR: malformed message, dropping data"
          return
        
        while x>math.pi:
            x -= math.pi*2
        while x<-math.pi:
            x += math.pi*2
        
        self.x.set([x])
        self.dx.set([dx])
        self.ddx.set([ddx])
        
net = nef.Network('Nonlinear Control', seed=1)

plant = net.add(Physics('plant'))

#control = net.add(NonlinearControl('control'))
#net.connect(plant.getOrigin('x'), control.getTermination('x'))
#net.connect(plant.getOrigin('dx'), control.getTermination('dx'))
#net.connect(plant.getOrigin('ddx'), control.getTermination('ddx'))

net.make_input('target', [1])
#net.connect('target', control.getTermination('desired'))


net.make('state', 300, 3, radius=2)
net.connect(plant.getOrigin('x'), 'state', index_post=0)
net.connect(plant.getOrigin('dx'), 'state', index_post=1)
net.connect(plant.getOrigin('ddx'), 'state', index_post=2)

net.make('s', 100, 1)
net.connect('state', 's', transform=[lambd, 1, 0])
net.connect('target', 's', weight=-lambd)

u = net.make('u', 100, 1, radius=1)
net.connect('s', 'u', weight=-kappa)

def learn(x):
    return [0]
net.connect('state', 'u', func=learn)

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
        
        
learn=net.add(Learn('learn', net.get('state').getOrigin('learn')))
net.connect('s', learn.getTermination('s'))
net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y'))  
        


#net.connect(control.getOrigin('u'), plant.getTermination('u'))
net.connect('u', plant.getTermination('u'))

def communication():
  sock = connect_port( PORT )
  if not sock:
    sys.exit(1)

  print( "sock connected" )
  print( "please press q to quit and use 8456 to move" )
  esc = 0
  _id = 0

  while not esc:
    c = "6" #getchar()
    speed = 0
    rot = 0
    if (c=="8"):
      speed = 0.1
    elif (c=="5"):
      speed = -0.1
    elif (c=="4"):
      rot = 0.1
    elif (c=="6"):
      rot = -0.1
    if (speed != 0 or rot != 0):
      data_out = "id%d human move [%f, %f]\n" % (_id, speed, rot)
      sent = sock.send(data_out)
      print ("SENT DATA (%d bytes): %s" % (sent, data_out))
      _id = _id + 1

    if c == "q":
      esc = 1

  sock.close()
  print("\nBye bye!")

def move_robot( u ):
  sock = connect_port( PORT )
  if not sock:
    sys.exit(1)

  print( "sock connected" )
  print( "please press q to quit and use 8456 to move" )
  esc = 0
  _id = 0
  #speed = 2
  rot = -1
  #speed = u.origin['X'].decoded_output.get_value()[0]
  #speed = np.array(u.get())[0]
  speed = u.getOrigin('X').getValues().getValues()[0]
  print (speed)

  while not esc:
    data_out = "id%d atrv.motion set_speed [%f, %f]\n" % (_id, speed, rot)
    sent = sock.send(data_out)
    #print ("SENT DATA (%d bytes): %s" % (sent, data_out))
    _id = _id + 1








"""
def odom_callback( motion_msg ):
  x.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.pose.pose.orientation.x ] ) ) 
  dx.origin['X'].decoded_output.set_value( np.float32( [ motion_msg.twist.twist.angular.x ] ) ) 
  ddx.origin['X'].decoded_output.set_value( np.float32( [ 0 ] ) ) # FIXME: get acceleration here

def main():
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
"""






#communication()

net.view()
net.add_to_nengo()        
#move_robot( u )
        
