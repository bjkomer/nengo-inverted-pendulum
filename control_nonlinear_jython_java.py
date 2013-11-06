lambd = 0.9
kappa = 5
rho = 0.2

import numeric as np
import math
import socket
import sys
sys.path.append('/home/bjkomer/Downloads/jyson-1.0.2/src')
sys.path.append('/home/bjkomer/Downloads/jyson-1.0.2/lib/jyson-1.0.2.jar')
sys.path.append('/home/bjkomer/nengo-inverted-pendulum')
import Learn
import com.xhaus.jyson.JysonCodec as json # Jython version of json
from com.xhaus.jyson import JSONDecodeError, JSONEncodeError

HOST = '127.0.0.1'
# TODO: these port values are not consistently the same in morse
PORT_ODOM = 60001
PORT_CONT = 60000

import nef
from nef.node import NodeTermination

#FIXME: temporary for tuning timing parameters
PHYSICS_PERIOD = 10 #10
LEARNING_PERIOD = 10 #50
import time #for timing data

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
        #"""
        self.sock_in = connect_port( PORT_ODOM ) # Reads odometry
        self.sock_out = connect_port( PORT_CONT ) # Outputs control signal (torque)
        if not self.sock_in or not self.sock_out:
          sys.exit(1)
        #"""
        self.odom_str = ""
        self.buf = ""
        self.sensor_data = { 'x' :0, 'y' :0, 'z' :0,
                             'vx':0, 'vy':0, 'vz':0,
                             'wx':0, 'wy':0, 'wz':0,
                             'roll':0, 'pitch':0, 'yaw':0 }

        self.x = self.make_output('x', dimensions=1)
        self.dx = self.make_output('dx', dimensions=1)
        self.ddx = self.make_output('ddx', dimensions=1)
        
        self.u = self.make_input('u', dimensions=1)
        
        self.counter = 0 # Used so that computations don't need to occur on every tick

        # For time benchmarking
        self.total_time = 0
        self.avg_time = 0
        #"""
        self.check_sockets()

        # Set the socket to nonblocking
        self.sock_in.setblocking(0)
        #"""
    def tick(self):

        self.counter += 1
        # Read in the angular position and velocity from the simulator
        
        # Burn through old messages, and only use the latest ones
        # This feels like a bad way to do this, but it works
        #"""
        while True:
          if self.read_socket( self.sock_in ):
            try:
              data_in = json.loads( self.odom_str )
            except JSONDecodeError:
              break
            self.sensor_data = data_in
          else:
            break
        #"""
        if self.counter % PHYSICS_PERIOD == 0:
          #t_start = time.time()
          dt = 0.001 * PHYSICS_PERIOD
          x = self.x._value[0]
          dx = self.dx._value[0]
          u = np.array(self.u.get())[0]
          #"""
          # Send u to the simulator as a torque
          #data_out = '{"force":[0,0,0],"torque":[%f,0,0]}\n' % (u * 4.6 )
          data_out = '{"force":[0,0,0],"torque":[%f,0,0]}\n' % u
          self.sock_out.send( data_out )
          #"""
          J = 0.1
          b = 0
          mgl = 0.1

          ddx = u/J + dx*abs(dx)*b/J + mgl*math.sin(x)/J
          #dx += ddx*dt
          #x += dx*dt    
          
          # FIXME: calculate acceleration properly
          self.x.set([self.sensor_data["roll"]])
          self.dx.set([self.sensor_data["wx"]])
          self.ddx.set([ddx])

    # Reads from a socket until it reaches a newline
    def read_socket( self, sock ):
      out = self.buf
      while True:
          try:
              data = sock.recv( 64 )
              # TODO: put meaningful error checking here
          except:
            return False
          if not data:
              break
          nl = data.find('\n')
          if nl >= 0:
              nl += 1
              out += data[:nl]
              self.buf = data[nl:]
              del data
              break
          out += data
      self.odom_str = out
      return True

    def check_sockets( self ):
      """
      Due to a bug with Blender and Ubuntu 13.04, socket ordering is
      non-deterministic. This function is a workaround to fix that, and swap the
      sockets if they are backwards. This can be done in a nicer way with pymorse,
      but it is not supported by jython
      """
      # Temporarily set timeout so it doesn't block
      self.sock_in.settimeout(1)
      if not self.read_socket( self.sock_in ):
        # If this socket timed out, it must not be for odometry.
        # They need to be flipped
        self.sock_in.settimeout( None )
        temp = self.sock_in
        self.sock_in = self.sock_out
        self.sock_out = temp

# TEMP - for testing
class SlowNetwork( nef.Network ):
  def __init__( self, name, seed=None ):
    nef.Network.__init__( self, name, seed )
    self.dt = 0.1

#TEMP
net = nef.Network('Nonlinear Control', seed=1)
#net = SlowNetwork('Nonlinear Control', seed=1)

plant = net.add(Physics('plant'))

#control = net.add(NonlinearControl('control'))
#net.connect(plant.getOrigin('x'), control.getTermination('x'))
#net.connect(plant.getOrigin('dx'), control.getTermination('dx'))
#net.connect(plant.getOrigin('ddx'), control.getTermination('ddx'))

net.make_input('target', [0])
#net.connect('target', control.getTermination('desired'))


state = net.make('state', 300, 3, radius=2)
#net.make('state', 150, 3, radius=2)
net.connect(plant.getOrigin('x'), 'state', index_post=0)
net.connect(plant.getOrigin('dx'), 'state', index_post=1)
net.connect(plant.getOrigin('ddx'), 'state', index_post=2)

s = net.make('s', 100, 1)
net.connect('state', 's', transform=[lambd, 1, 0])
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
        #self.Y = self.make_input('Y', dimensions=150, pstc=0.01)
        self.origin = origin
        self.counter = 0
        # added for testing
        self.total_time = 0
        self.avg_time = 0
    def tick(self):
        self.counter += 1
        if self.counter % LEARNING_PERIOD == 0: #10 FIXME
            #t_start = time.time()
            delta = -rho * np.array(self.s.get())*0.00001
            Y = np.array(list(self.Y.get()))
            Y.shape = 300,1
            #Y.shape = 150,1
            da = np.dot(Y, delta)
            ###decoder = np.array(self.origin.decoders)
            ###self.origin.decoders = decoder + da #FIXME: this line takes 50ms to run
            self.origin.decoders += da #FIXME: attempt to make it faster
            #self.total_time += time.time() - t_start
            #print( "learning: %f" % ( self.total_time / self.counter * LEARNING_PERIOD ) )
"""        
##s_learn = net.make('s_learn', neurons=100, dimensions=1 )
##Y_learn = net.make('Y_learn', neurons=300, dimensions=300)
##net.connect('state', 'Y_learn')
##net.connect('s', s_learn)
###net.connect(net.get('state').getOrigin('AXON'), Y_learn)  
####net.connect('state', Y_learn)  
#learn = Learn('learn_node', net.get('state').getOrigin('learn'), s_learn, Y_learn)
learn = Learn('learn_node', net.get('state').getOrigin('learn'), s, state)
#learn = Learn('learn_node', net.get('state').getOrigin('learn'), s, Y_learn)


net.add( learn )
#net.connect('s', learn.getTermination('s_learn'))
#net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y'))  
#net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y_learn'))  

#net.connect(control.getOrigin('u'), plant.getTermination('u'))
net.connect('u', plant.getTermination('u'))


net.view()
#net.run(30)
net.add_to_nengo()        
 
#while True:
#  net.run(1000, dt=0.001)
#net.run(1000, dt=0.02)
#net.run(1000, dt=0.1)
