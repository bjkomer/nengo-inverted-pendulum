lambd = 0.9
kappa = 5 / 5
rho = 0.2

import numeric as np
import math

import nef

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
        
        self.x.set([x])
        self.dx.set([dx])
        self.ddx.set([ddx])
        
        
net = nef.Network('Nonlinear Control', seed=1)

plant = net.add(Physics('plant'))

net.make_input('target', [1])

net.make('state', 300, 3, radius=2)
net.connect(plant.getOrigin('x'), 'state', index_post=0)
net.connect(plant.getOrigin('dx'), 'state', index_post=1)
net.connect(plant.getOrigin('ddx'), 'state', index_post=2)

net.make('s', 100, 1)
net.connect('state', 's', transform=[lambd, 1, 0])
net.connect('target', 's', weight=-lambd)

net.make('u', 100, 1, radius=1)
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
            delta = -rho * np.array(self.s.get())*0.00001 * .1
            Y = np.array(list(self.Y.get()))
            Y.shape = 300,1
            da = np.dot(Y, delta)
            da.shape = 300,1 #Bug fix
            decoder = np.array(self.origin.decoders)
            #print( "%.9f, %.9f, %.9f, %.9f" % ( decoder[0][0], decoder[13][0],
            #                                   decoder[26][0], decoder[85][0] ) )
            #print( decoder.shape )
            #print( da.shape )
            #print( (decoder + da).shape )
            self.origin.decoders = decoder + da

learn=net.add(Learn('learn', net.get('state').getOrigin('learn')))
net.connect('s', learn.getTermination('s'))
net.connect(net.get('state').getOrigin('AXON'), learn.getTermination('Y'))  
        
net.connect('u', plant.getTermination('u'))
     
net.view()
net.add_to_nengo()        
