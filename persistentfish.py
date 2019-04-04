# -*- coding: utf-8 -*-
"""
@author: Spencer
"""
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.animation as animation

class PersistentFish:
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

    xpos = zeros(size(time));               % Global x position (m)
    dxpos = zeros(size(time));              % Change in global x position (m/s)
    ypos = zeros(size(time));               % Global y position (m)
    dypos = zeros(size(time));              % Change in global y position (m/s)

    xpos(1) = 0;
    ypos(1) = 0;

    s = zeros(size(time));                  % Relative distance along curvilinear path
    psi = zeros(size(time));                % Global heading (rad)
    """
    def __init__(self,bounds,bound_X,bound_Y,sigma_u=0.059,theta_u=4.21,mu_u=0.1402,
                 sigma_w=2.85,theta_w=2.74,mu_w=-0.02,sigma_o=12,fc=0):
        self.bounds = bounds
        self.bound_X = bound_X
        self.bound_Y = bound_Y
        self.sigma_u,self.theta_u,self.mu_u,self.sigma_w=sigma_u,theta_u,mu_u,sigma_w
        self.theta_w,self.mu_w,self.sigma_o,self.fc = theta_w,mu_w,sigma_o,fc
        self.U = 0. 
        self.Omega = 0.
        self.xpos = 0.01 # position in middle of tank
        self.ypos = 0.01
        self.S = 0.
        self.psi = 0.
        
#    def findDistance(self,bounds,psi,xpos,ypos,sensitivity):
#        """
#        % Computes the distance to the boundaries assuming the PTW maintained its
#        % heading.
#        % bounds - array of bounds
#        % psi - current heading
#        % xpos, ypos - global position of PTW
#        % sensitivity - "look ahead distance (m)", functionally it's the length of the
#        %               line created from the PTW.
#        
#        % returns dw -- distance to nearest boundary assuming fish maintains heading
#        """
#        # Create ray from x/y position in the current heading
#        ds = 0.01   # resolution of ray
#        xline = [xpos]
#        yline = [ypos]
#        
#        for index in range(1,int(sensitivity/ds)):
#            xline.append(xline[index-1]+ds*math.cos(psi))
#            yline.append(yline[index-1]+ds*math.sin(psi))
#        
#        # Now that we have ray, loop over this line checking if it crosses 
#        # the boundaries of the tank. If so we compute the distance and return
#        # it to the main loop
#        
#        R , C = bounds.shape
#        crossing = False  # initialize as false
#        crossingX = np.nan
#        crossingY = np.nan
#        
#        dw = 1000 # set to high value initially so it has little affect if nothing
#                  # is detected
#        
#        for i in range(0,len(xline)):
#           for j in range(0,C):
#               if (np.abs(xline[i]-bounds[0,j]) <= ds*2) & (np.abs(yline[i]-bounds[1,j]) <= ds*2):
#                  crossing = True
#                  crossingX = bounds[0,j]
#                  crossingY = bounds[1,j]
#                  
#                  dw = math.sqrt((xpos-crossingX)**2+(ypos-crossingY)**2)
#                  break
#              
#           if crossing:
#               break
#        return dw
    
    def findDistance(self,bounds,xpos,ypos,psi):
        
        # Generate bound segments
        m = np.zeros((bounds.shape[0],1))
        for index in range(0,bounds.shape[0]):
            m[index] = (bounds[index][1]-bounds[index-1][1])/(bounds[index][0]-bounds[index-1][0])
        
        bound_segments = np.append(bounds,m,axis=1)
        # Uses point-slope form to find intersection of ray (fish heading) and the 
        # bounds of the tank. Assumes psi is between 0 and 2pi
        
        # Create ray describing current heading
        if psi<0:
            psi += 2*math.pi
        m_ray = math.tan(psi)
        
        x_intersect_arr = np.array([])
        y_intersect_arr = np.array([])
        
        for index in range(0,bound_segments.shape[0]):
            # Loop through each boundary segment and find intersection point
    #        P1x = bound_segments[index][0]
    #        P1y = bound_segments[index][1]
    #        m_bound = bound_segments[index][2]
    #        
    #        
    #        x_intersect = (m_ray*P0x - m_bound*P1x + P1y - P0y)/(m_ray-m_bound)
    #        y_intersect = m_ray*(x_intersect-P0x) + P0y
            
            # Shortcut because we know the tank boundaries will always be either 
            # vertical or horizontal. 
            if abs(bound_segments[index][2]) == 0:
                # Horizontal line described by y = num
                y_intersect = bound_segments[index][1] # will intersect at this y value
                x_intersect = (y_intersect + m_ray*xpos - ypos)/m_ray # point-slope solved for x
            elif math.isinf(bound_segments[index][2]):
                # Vertical line described by x = num
                x_intersect = bound_segments[index][0] # will intersect this x value
                y_intersect = m_ray*(x_intersect - xpos) + ypos # point-slope solved for y
            
            # Save values in array
            x_intersect_arr = np.append(x_intersect_arr,x_intersect)
            y_intersect_arr = np.append(y_intersect_arr,y_intersect)
        
        # Use intersection points to find distance
        distances = np.sqrt(np.square(x_intersect_arr-xpos)+np.square(y_intersect_arr-ypos))
        
        # Now we need to find the right quadrant of intersection point.
        quad_ray = math.floor(psi/(math.pi/2))+1
        if (psi == math.pi/2 and xpos > 0) or (psi == math.pi and ypos > 0) or (psi == 3*math.pi/2 and xpos < 0):
            # correct for weird instances where quadrant is iffy. Shouldn't ever occur
            # because it's very very unlikely yaw will be exactly pi, pi/2, etc.
            quad_ray -= 1
        
        if (psi == 0 and ypos < 0):
            # see above, yes these are one-off solutions but again this shouldn't really occur
            # in practice.
            quad_ray = 4
            
        intersect_angles = np.arctan2(y_intersect_arr-ypos,x_intersect_arr-xpos)
        for i in range(len(intersect_angles)):
            if intersect_angles[i] < 0:
                intersect_angles[i] += 2*math.pi
                
        quad_intersections = np.floor(intersect_angles/(math.pi/2))+1
        
        flag = 0
        for i in range(len(quad_intersections)):                        # not out of the fish bounds
            x_pt = x_intersect_arr[i]
            y_pt = y_intersect_arr[i]
            if ((quad_intersections[i] == quad_ray) and (abs(x_pt) <= (self.bound_X+0.1)) and (abs(y_pt) <= (self.bound_Y+0.1))):
                dw = distances[i]
#                x_intersect_actual = x_intersect_arr[i]
#                y_intersect_actual = y_intersect_arr[i]
                break
            else:
#                dw = math.sqrt((xpos-self.bound_width/2)**2+(ypos-self.bound_X/2)**2)
                dw = 0.1
                flag = 1
#                print("Error: Could not find intersection, defaulting to 0.09m\n")
#                print(str(distances)+"\n")
#                print(str(x_intersect_arr))
#                print(str(y_intersect_arr))
#                print(str(xpos)+"\t"+str(ypos))
#                print(str(psi)+"\n")
#                print(str(quad_ray)+"\n")
#                print(str(quad_intersections)+"\n")
#                print("----------------------------")
         
        if (flag == 1):
            print(str(xpos)+"\t"+str(ypos)+"\t"+str(psi))
#        print("----------------------------")
#        print(str(psi)+'\n')
#        print(str(distances)+'\n')
#        print(str(x_intersect_arr)+'\n')
#        print(str(y_intersect_arr)+'\n')
#        print(str(dw)+'\n')
        return dw
        
    def calcDerivatives(self,Omega,U,xpos,ypos,psi):
        """
        Calculates derivatives based on previous values of yaw rate (Omega) and
        forward speed (U). These stochastic differential equations also incorporate
        coupling function (fc) and wall function (fw).
        """
        theta_w,mu_w,sigma_w = self.theta_w,self.mu_w,self.sigma_w
        theta_u,mu_u,sigma_u = self.theta_u,self.mu_u,self.sigma_u
        
        sigma_o = self.sigma_o
        dt = self.dt
        
        # Compute coupling force fc
        fc = sigma_o*(2*sigma_o/sigma_w)**(-U/mu_u)
        
        # Compute wall force
        dw = PersistentFish.findDistance(self,self.bounds,xpos,ypos,psi);
#        fw = 2.25*math.exp(-0.11*dw)
        fw = 8*math.exp(-0.11*dw)
        
        if Omega >= 0:
            # Repulsive behavior, depending on sign of previous turning speed 
            # it'll push in either direction
            fw = -fw
        
        # Obtain random values that act as forcing terms.
        # randn() should work just like randn in matlab. Normally distributed about 0 mean
        dZ = np.random.randn()
        dW = np.random.randn()
        
        # Derivatives
        Omegadot = theta_w*(mu_w+fw-Omega)*dt + fc*dZ;
        Udot = theta_u*(mu_u-U)*dt + sigma_u*dW;
        
        return Omegadot, Udot, dw # return dw for detecting collision
    
    def updateStates(self,dt,Omega,U,xpos,ypos,psi,S):
        self.dt = dt
        Omegadot, Udot, dw = self.calcDerivatives(Omega,U,xpos,ypos,psi)
        self.Omega += Omegadot*dt
        self.U += Udot*dt
        
        if dw <= 0.1:
            self.U = 0
            
        # Determine relative positions S, psi
        self.S = S + self.U*dt
        self.psi = psi + self.Omega*dt
        if abs(self.psi)>=2*math.pi:
            # Keep yaw within 0 to 2pi
            if self.psi < 0:
                self.psi += 2*math.pi
            if self.psi > 0:
                self.psi -= 2*math.pi
        
        # Use these to transform to local coordinate
#        if((self.xpos < 0.05) or (self.xpos > (self.bound_X-0.05))):
#            self.xpos = xpos
#        else:
        self.xpos = self.xpos + self.U*math.cos(psi)*dt
            
#        if((self.ypos < 0.05) or (self.ypos > (self.bound_Y-0.05))):
#            self.ypos = ypos
#        else:
        self.ypos = self.ypos + self.U*math.sin(psi)*dt
        
        return self.Omega, self.U, self.xpos, self.ypos, self.psi, self.S
    
    def drivePersistentFish(self,dt):
        self.dt = dt
        # Update states given current position and speed
        self.updateStates(dt,self.Omega,self.U,self.xpos,self.ypos,self.psi,self.S)
#        print(str(self.xpos)+'\t'+str(self.ypos)+'\n')
#        print(str(self.psi)+'\n')
        return self.Omega,self.U,self.xpos,self.ypos,self.psi,self.S
        
### Main Function #############################################################
        

        
def main():

    ##### DEFINE BOUNDARIES (IN THIS CASE RECTANGULAR) ########################
#    bound_resolution = 0.01  # resolution of boundaries
#    bound_X = 1.2      # meters
#    bound_Y = 1.2
#    
#    xrange = np.arange(-bound_length/2,bound_length/2+bound_resolution,bound_resolution)
#    xrange = np.reshape(xrange,(1,-1))
#    yrange = np.arange(-bound_width/2,bound_width/2+bound_resolution,bound_resolution)
#    yrange = np.reshape(yrange,(1,-1))
#    
#    bound_topx = xrange
#    bound_topy = bound_width/2*np.ones(xrange.shape)
#    bound_botx = np.fliplr(xrange)
#    bound_boty = -bound_width/2*np.ones(xrange.shape)
#    bound_leftx = -bound_length/2*np.ones(yrange.shape)
#    bound_lefty = yrange
#    bound_rightx = bound_length/2*np.ones(yrange.shape)
#    bound_righty = np.fliplr(yrange)
#    
#    bounds_x = np.concatenate((bound_topx,bound_rightx,bound_botx,bound_leftx),axis=None)
#    bounds_y = np.concatenate((bound_topy,bound_righty,bound_boty,bound_lefty),axis=None)
#    
#    bounds = np.vstack((bounds_x,bounds_y))
    
    bound_X = 1.2 #x
    bound_Y = 1.2 #y
    
    bounds = np.array([[    -0.1,    -0.1],
                       [    -0.1,     bound_Y],
                       [     bound_X,     bound_Y],
                       [     bound_X,    -0.1]])
    
    ###########################################################################
    
    fish = PersistentFish(bounds,bound_X,bound_Y)
    
    simtime = 100 #seconds
    dt = 0.1
    
    t = np.arange(0,simtime,dt)
    
    fish_xpos = np.zeros((len(t),1))
    fish_ypos = np.zeros((len(t),1))    
    fish_Omega = np.zeros((len(t),1))
    fish_U = np.zeros((len(t),1))
    fish_psi = np.zeros((len(t),1))
    fish_S = np.zeros((len(t),1))
    
    file = open("class_testing_data.txt",'w')
    file.write("Time\txpos\typos\tpsi\tOmega\tU\tS\n")
    file.write(str(0)+"\t"+str(0)+"\t"+str(0)+"\t"+str(0)+"\t"+str(0)+"\t"+str(0)+"\t"+str(0)+"\n")
    for index in range(2,len(t)):
        # Calculate fish
        fish_Omega[index],fish_U[index],fish_xpos[index],fish_ypos[index],fish_psi[index],fish_S[index] = fish.drivePersistentFish(dt)
        file.write("%.3f" % t[index]+"\t")
        file.write("%.3f" % fish_xpos[index][0]+"\t")
        file.write("%.3f" % fish_ypos[index][0]+"\t")
        file.write("%.3f" % fish_psi[index][0]+"\t")
        file.write("%.3f" % fish_Omega[index][0]+"\t")
        file.write("%.3f" % fish_U[index][0]+"\t")
        file.write("%.3f" % fish_S[index][0]+"\n")
    
#    file.close()
#    
#    
#    fig, ax = plt.subplots()
#    line, = ax.plot(fish_xpos, fish_ypos, color='k')
#    
#    def update(num, x, y, line):
#        line.set_data(x[:num], y[:num])
#        line.axes.axis([bounds[0,0]-0.1, bound_X+0.1, bounds[0,1]-0.1, bound_Y+0.1])
#        return line,
#    plt.plot(np.append(bounds[:,0],bounds[0,0]),np.append(bounds[:,1],[bounds[0,1]]),'r-')
#    ani = animation.FuncAnimation(fig, update, len(fish_xpos), fargs=[fish_xpos, fish_ypos, line],
#                              interval=simtime, blit=True)
#    ani.save('test.mp4')
#    plt.show()
#    plt.close()

    plt.figure()
    plt.plot(fish_xpos,fish_ypos,'k-')
    plt.plot(np.append(bounds[:,0],bounds[0,0]),np.append(bounds[:,1],[bounds[0,1]]),'r-')
    plt.xlim((0,bound_X))
    plt.ylim((0,bound_Y))
    plt.axis('equal')
    plt.xlabel('Fish X Position (m)')
    plt.ylabel('Fish Y Position (m)')
    plt.title('Simulated Position')
    plt.show()
          
if __name__ == '__main__':
    main()
    
        
        
        
        
