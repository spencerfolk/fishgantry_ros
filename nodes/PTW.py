
from numpy import *
import math
import datetime



class PersistentFish():
    """
    Class defining a fish following persistent random turning behavior as 
    detailed in Zienkiewicz 2015 paper.
    sigma_u = 0.059;                        % (m/s)
    theta_u = 4.21;                         % (s^-1)
    mu_u = 0.1402;                          % mean speed (m/s)
    sigma_w = 2.85;                         % (rad/s)
    theta_w = 2.74;                         % (s^-1)
    mu_w = -0.02;                           % mean yaw rate (rad/s)
    fw = 0.0;                               % Forcing term due to boundaries
    dw = 0.0;                               % Magnitude of distance to boundary
    sigma_o = 12;                           % saturation variance (rad/s)
    fc = 0.0;                               % coupling function, forcing term
    
    U = zeros(size(time));                  % swimming speed array (m/s)
    dU = zeros(size(time));                 % change in swimming speed (m/s^2)
    Omega = zeros(size(time));              % yaw rate, changes w random input (rad/s)
    dOmega = zeros(size(time));             % change in yaw rate, acceleration, (rad/s^2)
    dW = randn(size(time));                 % random input to yaw rate, should be Brownian if possible
    dZ = randn(size(time));                 % random input to speed, should be Brownian if possible
    x = zeros(size(time));               % Global x position (m)
    dxpos = zeros(size(time));              % Change in global x position (m/s)
    y = zeros(size(time));               % Global y position (m)
    dypos = zeros(size(time));              % Change in global y position (m/s)
    x(1) = 0;
    y(1) = 0;
    s = zeros(size(time));                  % Relative distance along curvilinear path
    psi = zeros(size(time));                % Global heading (rad)
    """
    def __init__(self,sigma_u=0.3034,theta_u=14.6096,mu_u=0.0757, sigma_zdot=0.0873,mu_zdot=0.0114,theta_zdot=1,sigma_w=2.85,theta_w=2.74,mu_w=-0.02,sigma_o=0.1145,fc=0):
# sigma_u=0.3322,theta_u=13.6849,mu_u=0.0639, sigma_zdot=0.0873,mu_zdot=0.0114,theta_zdot=1,sigma_w=2.85,theta_w=2.74,mu_w=-0.02,sigma_o=12,fc=0                                                                                                    theta_zdot = 9.6423
    # ZIENK VALUES: self,sigma_u=0.059,theta_u=4.21,mu_u=0.1402, sigma_w=2.85,theta_w=2.74,mu_w=-0.02,sigma_o=12,fc=0
        self.sigma_u,self.theta_u,self.mu_u,self.sigma_w=sigma_u,theta_u,mu_u,sigma_w
        self.theta_w,self.mu_w,self.sigma_o,self.fc = theta_w,mu_w,sigma_o,fc
        self.sigma_zdot,self.mu_zdot,self.theta_zdot = sigma_zdot,mu_zdot,theta_zdot
        
        # Save these values for Markov switching
        self.sigma_u_save,self.theta_u_save,self.mu_u_save,self.sigma_w_save=sigma_u,theta_u,mu_u,sigma_w
        self.theta_w_save,self.mu_w_save,self.sigma_o_save,self.fc_save = theta_w,mu_w,sigma_o,fc
        self.sigma_zdot_save,self.mu_zdot_save,self.theta_zdot_save = sigma_zdot,mu_zdot,theta_zdot
        
        self.U = 0. 
        self.Omega = 0.
        self.yawrate = 0.
        self.x = 0. 
        self.y = 0.
        self.z = 0.
        self.zdot = 0.   # vertical speed (Vz)
        self.S = 0.
        self.psi = 0.
        self.Udot = 0

        self.theta = arange(0,2*pi,.01) #range of thetas
        self.tailtheta = 0
        self.tailangle = 0
        self.tailfreq = 0.
        self.maxfreq = 2*2*pi

        #for swimming up and down
        self.pitch = 0.
        self.pitchnow = 0
        self.laps = 0
        self.f = None
        self.maxamp = 1.5
        self.maxspeed = 0.1

        self.tailfreq_tau = 0.5
        self.pitchtau = 1.0
        # xmax = 1
        # ymax = 1
        # zmax = 1
        # self.updateGeometry(xmax,ymax,zmax)

    def updateGeometry(self,xmax,ymax,zmax):
        self.x = xmax/2
        self.y = ymax/2
        self.z = zmax/2

        self.bound_X = xmax
        self.bound_Y = ymax
        self.bound_Z = zmax
    
        self.bounds = array([[        0,                        0                 ],
                                [     0,                        self.bound_Y      ],
                                [     self.bound_X    ,         self.bound_Y      ],
                                [     self.bound_X    ,         0                 ]])

        print("updateGeometry successful - ")
        # print("Bounds = \n")
        # print(self.bounds);


    def findDistance(self,bounds,x,y,psi):
        
        if(psi >= (2*math.pi)):
            # findDistance commands only likes psi within 0-2pi so only for this instance use relative psi
            psi = psi - math.trunc(psi/(2*math.pi))*2*math.pi
            
        # Generate bound segments
        m = zeros((bounds.shape[0],1))
        for index in range(0,bounds.shape[0]):
            m[index] = (bounds[index][1]-bounds[index-1][1])/(bounds[index][0]-bounds[index-1][0])
        
        bound_segments = append(bounds,m,axis=1)
        # Uses point-slope form to find intersection of ray (fish heading) and the 
        # bounds of the tank. Assumes psi is between 0 and 2pi
        
        # Create ray describing current heading
        if psi<0:
            psi += 2*math.pi
        m_ray = math.tan(psi)
        
        x_intersect_arr = array([])
        y_intersect_arr = array([])
        
        for index in range(0,bound_segments.shape[0]):
            # Loop through each boundary segment and find intersection point
            # Shortcut because we know the tank boundaries will always be either 
            # vertical or horizontal. 
            if abs(bound_segments[index][2]) == 0:
                # Horizontal line described by y = num
                y_intersect = bound_segments[index][1] # will intersect at this y value
                x_intersect = (y_intersect + m_ray*x - y)/m_ray # point-slope solved for x
            elif math.isinf(bound_segments[index][2]):
                # Vertical line described by x = num
                x_intersect = bound_segments[index][0] # will intersect this x value
                y_intersect = m_ray*(x_intersect - x) + y # point-slope solved for y
            
            # Save values in array
            x_intersect_arr = append(x_intersect_arr,x_intersect)
            y_intersect_arr = append(y_intersect_arr,y_intersect)
        
        # Use intersection points to find distance
        distances = sqrt(square(x_intersect_arr-x)+square(y_intersect_arr-y))
        
        # Now we need to find the right quadrant of intersection point.
        quad_ray = math.floor(psi/(math.pi/2))+1
        if (psi == math.pi/2 and x > 0) or (psi == math.pi and y > 0) or (psi == 3*math.pi/2 and x < 0):
            # correct for weird instances where quadrant is iffy. Shouldn't ever occur
            # because it's very very unlikely yaw will be exactly pi, pi/2, etc.
            quad_ray -= 1
        
        if (psi == 0 and y < 0):
            # see above, yes these are one-off solutions but again this shouldn't really occur
            # in practice.
            quad_ray = 4
            
        intersect_angles = arctan2(y_intersect_arr-y,x_intersect_arr-x)
        for i in range(len(intersect_angles)):
            if intersect_angles[i] < 0:
                intersect_angles[i] += 2*math.pi
                
        quad_intersections = floor(intersect_angles/(math.pi/2))+1
        
        flag = 0
        for i in range(len(quad_intersections)):                        # not out of the fish bounds
            x_pt = x_intersect_arr[i]
            y_pt = y_intersect_arr[i]
            if ((quad_intersections[i] == quad_ray) and (abs(x_pt) <= (self.bound_X+0.1)) and (abs(y_pt) <= (self.bound_Y/2+0.1))):
                dw = distances[i]
#                x_intersect_actual = x_intersect_arr[i]
#                y_intersect_actual = y_intersect_arr[i]
                break
            else:
                dw = 0.1
                # flag = 1
        # if flag:
        #     print(str(x)+"\t"+str(y)+"\t"+str(psi))

        return dw
        
    def calcDerivatives(self,Omega,U,x,y,zdot,psi):
        """
        Calculates derivatives based on previous values of yaw rate (Omega) and
        forward speed (U). These stochastic differential equations also incorporate
        coupling function (fc) and wall function (fw).
        """

        
        sigma_o = self.sigma_o
        dt = self.dt

        theta_w,mu_w,sigma_w = self.theta_w,self.mu_w,self.sigma_w
        theta_u,mu_u,sigma_u = self.theta_u,self.mu_u,self.sigma_u*.033/dt
        theta_zdot,mu_zdot,sigma_zdot = self.theta_zdot,self.mu_zdot,self.sigma_zdot*.033/dt
        
        # Compute coupling force fc
#         if(U<mu_u):
#             fc=sigma_w
#         else:
#             fc=sigma_w
        fc = sigma_o*(2*sigma_o/sigma_zdot)**(-U/mu_u)
    
    
        # Compute wall force
        dw = PersistentFish.findDistance(self,self.bounds,x,y,psi);
        fw = 1.09*math.exp(-2.05*dw)
        
        if Omega >= 0:
            # Repulsive behavior, depending on sign of previous turning speed 
            # it'll push in either direction
            fw = -fw
        
        # Obtain random values that act as forcing terms.
        # randn() should work just like randn in matlab. Normally distributed about 0 mean
        dZ = random.randn()
        dW = random.randn()
        dB = random.randn()
        
        # Derivatives
        # Omegadot = theta_w*(mu_w+fw-Omega)*dt + sigma_w*dZ
        Omegadot = theta_w*(mu_w-Omega)*dt + sigma_w*dZ
        # Omegadot = sigma_w*dZ
        Udot = theta_u*(mu_u-U)*dt + fc*dW
        self.Udot = Udot
        zdoubledot = theta_zdot*(mu_zdot-zdot)+sigma_zdot*dB
        
        return Omegadot, Udot, zdoubledot, dw # return dw for detecting collision
    
    def updateStates(self,dt,Omega,U,x,y,zdot,psi,S,state=1):
        self.dt = dt
        Omegadot, Udot, zdoubledot, dw = self.calcDerivatives(Omega,U,x,y,zdot,psi)
        self.Omega += Omegadot*dt
        # if state == 1:
        #     self.U += Udot*dt
        # else:
        #     self.U = 0
        self.U += Udot*dt
        self.zdot += zdoubledot*dt
        
        if(self.U<0):
            # print "U can't do that!"
            self.U=0
        
        # if dw <= 0.005:
        #     self.U = 0
            
        ##### Bound z by setting zdot = 0 when approaching boundaries
        ##### TOP - z = 0, anything less than 0 will stop it
        ##### BOTTOM - z = maxz anything more will stop it.
        ##### Recall that positive zdot is down
        if((self.z<=0.001) and (self.zdot<0)):
            # If we're at the min depth (0) and going upwards (zdot < 0)
            self.zdot = 0
#        elif((self.z>=(app.zmax-0.001)) and (self.zdot>0)):  # in case bound_Z isn't updated for some reason.. pull zmax from Window class.
        elif((self.z>=(self.bound_Z-0.001)) and (self.zdot>0)):
            self.zdot = 0
        
        # Determine relative positions S, psi
#        self.S += self.U*dt  ######## NOTE THIS IS JUST IN-PLANE PATH
        #                   in-plane path     depth path
        self.S += math.sqrt((self.U*dt)**2 + (self.zdot*dt)**2)
        self.psi = psi + self.Omega*dt
        
        # laps can be found by dividing current psi by 2pi to find number of laps
        psi_sign = self.psi/abs(self.psi)  # if negative this will return negative, positive positive.
        self.laps = psi_sign*math.trunc(abs(self.psi)/(2*math.pi))
        
#        if abs(self.psi)>=2*math.pi:
#            # Keep yaw within 0 to 2pi
#            if self.psi < 0:
#                self.psi += 2*math.pi
#            if self.psi > 0:
#                self.psi -= 2*math.pi

################## SF WALL LIMITS ###########################################
        if ((self.x <= 0.01) or (self.x >= self.bound_X - 0.01)):
            # If the fish is outside left/right bounds
            if((self.x<=0.01) and (self.U*math.cos(self.psi) <= 0)):
                # If it's on the left wall and moving left
                self.x = self.x
            elif((self.x<=0.01) and (self.U*math.cos(self.psi)> 0)):
                self.x = self.x + self.U*math.cos(self.psi)*dt
                
            if((self.x>=(self.bound_X - 0.01)) and (self.U*math.cos(self.psi) >= 0)):
                # If it's on the right wall and moving right
                self.x = self.x
            elif((self.x>=(self.bound_X - 0.01)) and (self.U*math.cos(self.psi) < 0)):
                self.x = self.x + self.U*math.cos(self.psi)*dt
        else: 
            self.x = self.x + self.U*math.cos(self.psi)*dt
            
        if ((self.y <= 0.01) or (self.y >= self.bound_Y - 0.01)):   
            # If it's outside top/bottom bounds
            if((self.y<=0.01) and (self.U*math.sin(self.psi) <= 0)):
                # If it's at bottom wall and moving down
                self.y = self.y
            elif((self.y<=0.01) and (self.U*math.sin(self.psi) > 0)):
                self.y = self.y + self.U*math.sin(self.psi)*dt
            
            if((self.y>=(self.bound_Y - 0.01)) and (self.U*math.sin(self.psi) >= 0)):
                # If it's at top wall and moving up
                self.y = self.y
            elif((self.y >= (self.bound_Y - 0.01)) and (self.U*math.sin(self.psi) < 0)):
                self.y = self.y + self.U*math.sin(self.psi)*dt
        else:
            self.y = self.y + self.U*math.sin(self.psi)*dt

################### AAB WALL LIMITS #########################################
#         if(self.x<.01):
#             if(self.U*cos(self.psi)<=0):
#                 print"bounding x"
#                 self.x=self.x
#             else:
#                 self.x = x + self.U*math.cos(psi)*dt
#         else:
#             self.x = x + self.U*math.cos(psi)*dt
#         if(self.x>(self.bound_X-.01)):
#             if(self.U*cos(self.psi)>=0):
#                 print "bounding x for bigbound"
#                 self.x=self.x
#             else:
#                 self.x = self.x + self.U*math.cos(psi)*dt
#         else:
#             self.x = self.x + self.U*math.cos(psi)*dt

#         if(self.y<.01):
#             if(self.U*sin(self.psi)<=0):
#                 print "bounding y"
#                 self.y=self.y
#             else:
#                 self.y = self.y + self.U*math.sin(psi)*dt
#         else:
#             self.y = self.y + self.U*math.sin(psi)*dt
#         if(self.y>(self.bound_Y-.01)):
#             if(self.U*sin(self.psi)>=0):
#                 self.y=self.y
#                 print "bounding y for bigbound"
#             else:
#                 self.y = self.y + self.U*math.sin(psi)*dt
#         else:
#             self.y = self.y + self.U*math.sin(psi)*dt
############################################################################
        
        # # Use these to transform to local coordinates
        # if ((self.x<.01) or (self.x>(self.bound_X-.01))):
        #     if(self.U*)
        #     self.x = self.x
        # else:
        #     self.x = x + self.U*math.cos(psi)*dt
        # if (  (self.y<.01) or (self.y>(self.bound_Y - .01))):
        #     self.y = self.y
        # else:
        #     self.y = self.y + self.U*math.sin(psi)*dt
            
        self.z = self.z + self.zdot*dt

        # self.y = y + self.U*math.sin(self.psi)*dt
        # self.x = x + self.U*math.cos(self.psi)*dt

        # tail+dt*self.Uuff
        # self.pitchnow = 0.
        if(self.Udot>0):
            tailfreq_new = self.maxfreq*self.Udot/(self.sigma_u)*7.5
        else:
            tailfreq_new = 0

        if self.statusNow == "Inactive":
            tailfreq_new = 0

        self.tailfreq = (1-dt/self.tailfreq_tau)*self.tailfreq+dt/self.tailfreq_tau*tailfreq_new

        self.tailtheta+=self.tailfreq*dt
        self.tailangle = self.maxamp*sin(self.tailtheta) - 2*self.maxamp*self.yawrate
        if(U>.01):
            pitchnew = -arctan2(self.zdot,self.U)
        else:
            pitchnew = 0

        self.pitchnow = (1-dt/self.pitchtau)*self.pitchnow+dt/self.pitchtau*pitchnew

        # print self.pitchnow
        #print self.psi
        return self.x, self.y, self.psi, self.z, self.pitchnow, self.tailangle 
        #return Omega, U, S, psi, x, y

    
    def drivePersistentFish(self,dt,state=1):
        self.dt = dt
        # Update states given current position and speed
        self.updateStates(dt,self.Omega,self.U,self.x,self.y,self.zdot,self.psi,self.S,state)
#        print(str(self.x)+'\t'+str(self.y)+'\n')
#        print(str(self.psi)+'\n')

        # return self.Omega,self.U,self.x,self.y,self.psi,self.S
        return self.x,self.y,-self.z,self.pitchnow,self.psi,self.tailangle

class MarkovChain(PersistentFish):
    # Inherits a persistentfish and runs a 2-state Markov chain with time-dependent transition matrices.
    def __init__(self):
        # self.tau_active = 4.5846
        # self.tau_inactive = 1.7966
        PersistentFish.__init__(self)  # Initialize markov as a child of PersistentFish

        # Initialize states, transition names and their conditions, etc.
        self.elapsed = 0.

        self.states = ["Active","Inactive"]
        #                       # A - Active , I - Inactive , ex: AI = from Active to Inactive
        self.transitionNames = [["AA","AI"],["II","IA"]]

        self.transitionMatrix = [[0.8073,0.1927],[0.7537,0.2463]]

        # Check transition matrix to make sure we're doin ok. Each row should add to 1
        if ((sum(self.transitionMatrix[0]) == 1) and (sum(self.transitionMatrix[1]) == 1)):
            print "Markov Chain established"
        else:
            print "Markov Chain initialized improperly, try again"
            print self.transitionMatrix

        self.statusNow = "Active"

#   def updateTransitionMatrix(self):
#       # inputting dt will compute transition/probability matrix (which is fed into selecting which state to go into)
#       # Structure of transitions:  [["AA","AI"],["II","IA"]]
#       # Active = A, Inactive = I
#       self.transitionMatrix = [[math.exp(-self.elapsed/self.tau_active),(1-math.exp(-self.elapsed/self.tau_active))],[math.exp(-self.elapsed/self.tau_inactive),(1-math.exp(-self.elapsed/self.tau_inactive))]]
#       return self.transitionMatrix

    def updateCurrentState(self):
#       self.transitionMatrix = self.updateTransitionMatrix()
        if self.statusNow == "Active":
            change = random.choice(self.transitionNames[0],replace=True,p=self.transitionMatrix[0])
            if change == "AI":
                self.statusNow = "Inactive"
                self.setInactive()
                print "State changed from Active to Inactive"
#               print self.elapsed
#               self.elapsed = 0  # reset elapsed
        if self.statusNow == "Inactive":
            change = random.choice(self.transitionNames[1],replace=True,p=self.transitionMatrix[1])
            if change == "IA":
                self.statusNow = "Active"
                self.setActive()
                print "State changed from Inactive to Active"
#               print self.elapsed
#               self.elapsed = 0  # reset elapsed
                
    def setActive(self):
        # Use temp values
        self.sigma_u, self.theta_u, self.mu_u, self.sigma_w = self.sigma_u_save, self.theta_u_save, self.mu_u_save, self.sigma_w_save
        self.theta_w, self.mu_w, self.sigma_o, self.fc =  self.theta_w_save, self.mu_w_save, self.sigma_o_save, self.fc_save
        self.sigma_zdot, self.mu_zdot, self.theta_zdot =  self.sigma_zdot_save, self.mu_zdot_save, self.theta_zdot_save
        
    def setInactive(self):
        # Set inactive behavior
        
        # 0 for now so that it doesn't do anything
        self.sigma_u, self.theta_u, self.mu_u, self.sigma_w= 0.00001, self.sigma_u_save, 0.00001 , 0.00001
        self.theta_w, self.mu_w, self.sigma_o, self.fc = 10*self.theta_w_save, 0.00001, 0.0001, 0.00001
        self.sigma_zdot, self.mu_zdot, self.theta_zdot = 0.00001, 0.00001, 0.00001

        self.U = 0 # stop the fish
