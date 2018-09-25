from numpy import *
from matplotlib.pyplot import *

class TailCommandGenerator:
    def __init__(self):
        self.phi = 0
        self.theta_ref = 0

    def update(self,enable,freq,amp,bias,dt):
        if enable==1:
            self.phi+=freq*dt
            self.theta_ref = bias+amp*sin(self.phi)

class TailServo:
    def __init__(self,tau=0.03,wmax=pi/3*10,wn=2*pi*10,zeta=0.8):
        """
        TailServo(wn,zeta,wmax)
        class that simulates a servomotor-based tail that follows a second order TF with a slew.

        initial parameters are designed to mimic a traxxas 2065.

        omega(s)/V(s) = A/(s+a) standard motor velocity profile.

        A/a = omegamax/V. For generality, assume voltage = 5. Really, it might not be 5, but who's counting.
        We will assume that this servo takes a position command, and wraps a PD controller around the motor.

        tau will be about .03 seconds (guess), and a = 1/tau

        """

        self.tau,self.wmax,self.wn,self.zeta=tau,wmax,wn,zeta
        self.a = 1/self.tau
        self.A = self.a*wmax/5.0

        #now let's look at wrapping a PD controller around this.
        #theta(s)/v(s) = A/(s(s+a))

        #theta(s) * (s^2+a*s) = v(s)*A
        #thetaddot + a*thetadot = v*A
        #thetaddot = -a*thetadot + A*kp(r-theta) + A*kd(rdot-thetadot)
        #then the characteristic equation is:
        #s^2+(A*kd+a)s+A*kp=0

        #wn^2 = A*kp so...
        self.kp = self.wn**2/self.A
        #2*zeta*wn = (A*kd+a)
        self.kd = (2*self.zeta*self.wn-self.a)/self.A

        self.x = array([0.,0.])
        self.xdot = array([0.,0.])


    def calcDerivs(self,r):
        theta = self.x[0]
        thetadot = self.x[1]
        V = self.kp*(r-theta) - self.kd*thetadot
        if abs(V)>5.0:
            V=sign(V)*5
        thetaddot = -self.a*thetadot +self.A*V
        self.xdot = array([thetadot,thetaddot])
        return self.xdot

    def EulerUpdateStates(self,r,dT):
        xdot = self.calcDerivs(r)
        self.x+=xdot*dT
        return self.x

    def calcStepResponse(self,magnitude,simtime,dT):
        xsave = self.x
        xdotsave = self.xdot
        self.x = array([0.,0.])
        self.xdot = array([0.,0.])
        t = arange(0,simtime,dT)
        x = zeros((len(t),2))
        for ind in range(0,len(t)):
            x[ind,:] = self.EulerUpdateStates(magnitude,dT)
        self.x = xsave
        self.xdot = xdotsave
        return t,x


class RigidFish:
    def __init__(self,S=.01,Cd=.6418,Kd=6.5e-4,Cl=3.41,mb=.311,J=1.08e-4,L=.02,rho=1000.,m=.4909,c=3.25*.0254,kf=.918,a=-.0116,b=.411,fastcoast=True,fastcoast_tau = 0.2,height_tau = 0.5, tilt_tau = 0.2):
        """ 
        RigidFish(S=.01,Cd=.4418,Kd=6.5e-4,Cl=3.41,mb=.311,J=5.08e-4,L=.04,rho=1000.,m=.4909,c=0.07,kf=.918,a=-.0116,b=.411)
        Model adapted from Tan (2013) Chinese Control Conference
        S: frontal area
        Cd: Drag coeff
        Kd: Drag moment coeff
        Cl: lift Coefficient
        mb: mass
        J: yaw moment of inertia
        L: Fish length
        rho: density of water
        m: virtual mass (water-added) per unit length of tail
        c: distance from tail hinge to center of mass
        kf: scaling force Coefficient
        a: scaling moment Coefficient
        b: another(?) scaling moment Coefficient

        This is a state-space model of the open-loop behavior of a fish robot moving in-plane.

        state vector is x=[u,v,w,X,Y,psi]^T
        where u is local forward velocity, v is local lateral velocity, w is angular velocity in yaw. X, Y are global positions, psi is yaw angle.

        methods: 

        calcDerivs(alpha,alphaddot): calculates state derivatives based on current state and tail motion returns: xdot

        EulerUpdateStates(alpha,alphaddot,dT): calculates new states based on object's current state and inputs, and timestep. 
        """
        self.S,self.Cd,self.Kd,self.Cl,self.mb,self.J,self.L,self.rho,self.m,self.c,self.kf,self.a,self.b=S,Cd,Kd,Cl,mb,J,L,rho,m,c,kf,a,b
        self.x = array([0.,0.,0.,0.,0.,0.])
        self.fastcoast = fastcoast
        self.fastcoast_tau = fastcoast_tau#how fast is the 'active coast'?
        self.tail = TailServo()
        self.tail_desired = 0
        self.tailtheta_desired = 0
        self.coast_thresh = 0.5#how slow does freq need to be to indicate that the fish should be stopping?
        self.tailfreq = 1000#have to keep track of this to know whether we should coast
        self.height_tau = height_tau
        self.tilt_tau = tilt_tau

        self.tilt_command = 0
        self.tilt = 0
        self.height_command = 0
        self.height = 0


    def updateTilt(self,tiltcommand,dT):
        self.tilt_command = tilt_command
        self.tilt += dT/self.tilt_tau*(self.tiltcommand-self.tilt)

    def updateHeight(self,heightcommand,dT):
        self.height_command = height_command
        self.height += dT/self.height_tau*(self.heightcommand-self.height)

    def calcDerivs(self,alpha,alphaddot):
        #pull out state variables self.S,self.Cd,self.Kd,self.Cl,self.mb,self.J,self.L,self.rho,self.m,self.c,self.kf,self.a,self.band parameters for readability
        u,v,w,X,Y,psi = self.x
        S,Cd,Kd,Cl,mb,J,L,rho,m,c,kf,a,b=self.S,self.Cd,self.Kd,self.Cl,self.mb,self.J,self.L,self.rho,self.m,self.c,self.kf,self.a,self.b
        #calculate some parameters
        c1 = m/(2*mb)*L**2
        c2 = L**2/(2*J)*m*c
        c3 = Kd/J
        c4 = L**3/(3*J)*m
        #drag equations
        fu = -1/(2*mb)*rho*S*Cd*u*sqrt(u**2+v**2)+1/(2*mb)*rho*S*Cl*v*sqrt(u**2+v**2)*arctan2(v,u)
        fv = -1/(2*mb)*rho*S*Cd*v*sqrt(u**2+v**2)-1/(2*mb)*rho*S*Cl*u*sqrt(u**2+v**2)*arctan2(v,u)
        #state derivatives
        if(self.tailfreq>=self.coast_thresh):
            udot = v*w+fu - c1*alphaddot*sin(alpha)
            vdot = -v*w+fv + c1*alphaddot*cos(alpha)
            wdot = -c3*w**2*sign(w) - c2*alphaddot*cos(alpha)-c4*m*alphaddot
        else:#if frequency is low, assume the fish 'wants' to stop. Would do so with pectorals, etc. in real life.
            udot = -1.0/self.fastcoast_tau*self.x[0]
            vdot = -1.0/self.fastcoast_tau*self.x[1]
            wdot = -1.0/self.fastcoast_tau*self.x[2]
            #print("coasting")
        
        Xdot = u*cos(psi) - v*sin(psi)
        Ydot = u*sin(psi) + v*cos(psi)
        psidot = w
        return array([udot,vdot,wdot,Xdot,Ydot,psidot])

    def EulerUpdateStates(self,alpha,alphaddot,dT,tiltcomand = 0,heightcommand=0):
        self.dT = dT
        xdot = self.calcDerivs(alpha,alphaddot)
        self.x+=xdot*dT
        self.updateTilt(tiltcommand,dT)
        self.updateHeight(heightcommand,dT)
        return self.x

    def driveFish(self, freq,amp,bias,dT,enable=1):
        self.dT = dT
        self.tailfreq = freq
        if(enable==1):
            self.tailtheta_desired+=freq*dT
        else:
            pass
        self.tail_desired = bias+amp*sin(self.tailtheta_desired)
        #first calculate servo position
        self.tail.EulerUpdateStates(self.tail_desired,dT)
        #now calculate fish  
        self.EulerUpdateStates(self.tail.x[0],self.tail.xdot[1],dT)
        return self.x #return fish state vector in this class

def demoOpenLoop():
    print "this is the demo of the rigidfish class"

    tail = TailServo()
    tserv,xserv = tail.calcStepResponse(pi/3.,0.3,.0001)
    figure()
    plot(tserv,xserv[:,0]*180/pi,'k')
    xlabel('time (s)')
    ylabel('servo angle (deg)')
    title('step response of our tail servo')
    #show()

    fish = RigidFish()

    simtime = 1#seconds
    dT = 0.001 #seconds, timestep for simulation
    t = arange(0,simtime,dT) #time vector

    tail_freq = 2*pi*1.
    tail_bias = 40*pi/180
    tail_amp = 40*pi/180

    #create a vector of desired position
    tail_desired = tail_bias + tail_amp*sin(tail_freq*t)

    fish_x = zeros((len(t),6))
    tail_x = zeros((len(t),2))

    for ind in range(1,len(t)):
        #first calculate servo position
        tail_x[ind,:] = tail.EulerUpdateStates(tail_desired[ind],dT)
        #now calculate fish  
        fish_x[ind,:] = fish.EulerUpdateStates(tail_x[ind,0],tail.xdot[1],dT)
    
    figure()
    plot(fish_x[:,3],fish_x[:,4],'k')
    axis('equal')
    xlabel('Fish X (m)')
    ylabel('Fish Y (m)')

    figure()
    plot(t,tail_desired*180/pi,'r',t,tail_x[:,0]*180/pi,'k')
    xlabel('Time (s)')
    ylabel('Tail angle (deg)')
    show()

def demoDriveFish():
    print "this is the demo of the rigidfish class"


    fish = RigidFish()

    simtime = 10#seconds
    dT = 0.001 #seconds, timestep for simulation
    t = arange(0,simtime,dT) #time vector

    tail_freq = 2*pi*1.
    tail_bias = 40*pi/180
    tail_amp = 40*pi/180

    fish_x = zeros((len(t),6))

    for ind in range(1,len(t)):
        #now calculate fish  
        if((t[ind])<=(simtime/2.0)):
            fish_x[ind,:] = fish.driveFish(tail_freq,tail_amp,tail_bias,dT,1)
        else:
            fish_x[ind,:] = fish.driveFish(0,tail_amp,tail_bias,dT,1)
    
    figure()
    plot(fish_x[:,3],fish_x[:,4],'k')
    axis('equal')
    xlabel('Fish X (m)')
    ylabel('Fish Y (m)')
    
    figure()
    plot(t,fish_x[:,0],'k',t,fish_x[:,1],'r')
    #axis('equal')
    xlabel('Fish X (m)')
    ylabel('Fish Y (m)')

    show()



if __name__ == '__main__':
    demoDriveFish()