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

# NOTE: these port values are not consistently the same in morse
PORT_ODOM = 60001
PORT_CONT = 60000

CONTROL_PERIOD = 10 # determines how often to update the torque command

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
        
        self.buf = ""
        self.odom_str = "" # most recent odometry information (raw)
        
        # data from the current iteration
        self.sensor_data = { 'x' :0, 'y' :0, 'z' :0,
                             'vx':0, 'vy':0, 'vz':0,
                             'wx':0, 'wy':0, 'wz':0,
                             'roll':0, 'pitch':0, 'yaw':0 }
        
        # Connect with MORSE through a socket
        self.sock_in = connect_port( PORT_ODOM ) # Reads odometry
        self.sock_out = connect_port( PORT_CONT ) # Outputs control signal (torque)
        if not self.sock_in or not self.sock_out:
          sys.exit(1)

        self.position = self.make_output('position', dimensions=1)
        self.velocity = self.make_output('velocity', dimensions=1)
        
        self.torque = self.make_input('torque', dimensions=1)
        
        self.counter = 0 # Used so that computations don't need to occur on every tick
        
        self.check_sockets()

        # Set the socket to nonblocking
        self.sock_in.setblocking(0)

    def tick(self):

        # Burn through old messages, and only use the latest ones
        # This will allow the controller to keep working if the simulation slows
        # down. In the future they should be synchronized
        while True:
          if self.read_socket( self.sock_in ):
            try:
              data_in = json.loads( self.odom_str )
            except JSONDecodeError:
              break
            self.sensor_data = data_in
          else:
            break
        
        self.position.set([self.sensor_data["roll"]])
        self.velocity.set([self.sensor_data["wx"]])
        
        self.counter += 1
        if self.counter % CONTROL_PERIOD == 0:
          torque = np.array(self.torque.get())[0]
          
          # Send the command to the simulator as a torque
          data_out = '{"force":[0,0,0],"torque":[%f,0,0]}\n' % torque
          self.sock_out.send( data_out )

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

# set up the nengo network
net=nef.Network('Pendulum')

net.make_input('setpoint', [0])

plant = net.add(Physics('plant'))

torque_population = net.make('Torque', neurons=300, dimensions=1) # population for the output torque

net.connect( 'setpoint', 'Torque', weight=-1.0 )
net.connect(plant.getOrigin('position'), 'Torque', weight=-0.9)
net.connect(plant.getOrigin('velocity'), 'Torque', weight=-0.2)
net.connect('Torque', plant.getTermination('torque'))

net.view()
net.add_to_nengo()        
