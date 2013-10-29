# Controller for the morse simulation ( different message formats than the Gazebo one currently )
# This one uses the jython nengo

import numeric as np
import math
import socket
import sys
sys.path.append('/home/bjkomer/Downloads/jyson-1.0.2/src')
sys.path.append('/home/bjkomer/Downloads/jyson-1.0.2/lib/jyson-1.0.2.jar')
import com.xhaus.jyson.JysonCodec as json # Jython version of json
from com.xhaus.jyson import JSONDecodeError, JSONEncodeError

HOST = '127.0.0.1'
# TODO: these port values are not consistently the same in morse
PORT_ODOM = 60001
PORT_CONT = 60000

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

class Physics(nef.Node):
    def __init__(self, name):
        nef.Node.__init__(self, name)
        
        # Connect with MORSE through a socket
        self.sock_in = connect_port( PORT_ODOM ) # Reads odometry
        self.sock_out = connect_port( PORT_CONT ) # Outputs control signal (torque)
        if not self.sock_in or not self.sock_out:
          sys.exit(1)

        self.position = self.make_output('position', dimensions=1)
        self.velocity = self.make_output('velocity', dimensions=1)
        
        self.torque = self.make_input('torque', dimensions=1)
        
        self.counter = 0 # Used so that computations don't need to occur on every tick

    def tick(self):

        self.counter += 1
        if self.counter % 10 == 0: #FIXME == 0
          dt = 0.001 * 10
          position = self.position._value[0]
          velocity = self.velocity._value[0]
          torque = np.array(self.torque.get())[0]
          
          # Send u to the simulator as a torque
          #data_out = '{"force":[0,0,0],"torque":[%f,0,0]}\n' % (torque * 42 )
          data_out = '{"force":[0,0,0],"torque":[%f,0,0]}\n' % (torque * 10 )
          self.sock_out.send( data_out )

          # Read in the angular position and velocity from the simulator
          
          morse = self.sock_in.makefile("r")
          try:
            data_in = json.loads(morse.readline())
            position = data_in["roll"]
            velocity = data_in["wx"]
          except JSONDecodeError:
            #print "ERROR: malformed message, dropping data, trying again"
            try:
              data_in = json.loads(morse.readline())
              position = data_in["roll"]
              velocity = data_in["wx"]
            except JSONDecodeError:
              print "ERROR: malformed message, dropping data, skipping cycle"
              return
          
          self.position.set([position])
          self.velocity.set([velocity])

# set up the nengo network
net=nef.Network('Pendulum')
#net.add_to_nengo()

net.make_input('setpoint', [0])

plant = net.add(Physics('plant'))

torque_population = net.make('Torque', neurons=300, dimensions=1) # population for the output torque

net.connect( 'setpoint', 'Torque', weight=-1.0 )
net.connect(plant.getOrigin('position'), 'Torque', weight=-0.9)
net.connect(plant.getOrigin('velocity'), 'Torque', weight=-0.2)
net.connect('Torque', plant.getTermination('torque'))

net.view()
net.add_to_nengo()        
