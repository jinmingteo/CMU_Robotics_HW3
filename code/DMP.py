import numpy as np
import math

import matplotlib.pyplot as plt 

class DMP_trajectory_generator:
    def __init__(self, num_basis):
        '''
        DMP trajectory generator class
        inputs:
        num_basis = number of Gaussian basis functions to use
        '''
        self.T = 3 # value in seconds (time length of original trajectory)
        self.num_basis = num_basis
        self.K = 25*25/4
        self.B = 25
        self.dt = 0.003 # time step of trajectory
        self.C = np.linspace(0,1,self.num_basis) #Basis function centers
        self.H = (0.65*(1./(self.num_basis-1.))**2) #Basis function widths

    def learn_weights_from_raw_trajectory(self, q, dq,ddq):
        '''
        This function learns DMP weights (i.e. shape parameters) for the trajectory.

        inputs: q (raw joint trajectory), dq (joint velocities), ddq (joint accelerations) are from the raw trajectory provided to you.
        output: learned weights
        '''
        Time = np.linspace(0,self.T,1000)
        T=Time[-1]
        PHI= []
        F =[]
        for k in range(len(Time)): # Num of steps
            Phi=[math.exp(-0.5*((Time[k]/Time[-1])-c)**2 /self.H) for c in self.C] # Gaussian basis fn
            Phi=Phi/np.sum(Phi)
            PHI.append(Phi)
            
            f=((ddq[k]*T**2)-self.K*(q[-1]-q[k])+self.B*(dq[k]*T))/(q[-1]-q[0]) # Shuffling of acceleration eqn
            F.append(f)

        # calculate weights via linear regression
        w = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(PHI),PHI)),np.transpose(PHI)),F)

        self.weights = w

        return self.weights 

    def generate_dmp_trajectory(self):
        '''
        TODO: implement this method to use your learned weights to generate a smooth
        DMP trajectory.       

        output: dmp trajectory based on starting position and learned weights
        '''
        q0 = np.zeros(5) # starting joint positions
        cur_q = q0
        q_goal = np.array([0,0,0.7,3,1]) # goal joint positions - feel free to play around with this!
        qd = np.zeros(5)
        qdd = np.zeros(5)

        t = 0
        dmp_trajectory = {'time': [0], 'q': [q0]}
        
        for i in range(2000):
            t = t + self.dt
            # TODO: Write your code here
            if t <= self.T:
                Phi=[math.exp(-0.5*((t/self.T)-c)**2 /self.H) for c in self.C] # Gaussian basis fn
                Phi=Phi/np.sum(Phi)
                f=np.dot(Phi, self.weights)
            else:
                f= np.zeros(5)
            
            qdd = (self.K * (q_goal - cur_q) /self.T**2) - (self.B*qd/self.T) + ((q_goal - q0) * f / self.T**2)
            qd = qd + qdd * self.dt
            cur_q = cur_q + qd * self.dt
            
            dmp_trajectory['time'].append(dmp_trajectory['time'][-1] + self.dt)
            dmp_trajectory['q'].append(cur_q)
            
        return dmp_trajectory


if __name__ == "__main__":  
    q = np.load('q.npy') # raw joint trajectory
    dq = np.load('dq.npy')
    ddq = np.load('ddq.npy')

    # TODO: Fit weights to the provided trajectory and use the learned weights to generate a DMP trajectory
    # TODO: Plot your DMP trajectory
    
    num_basis_list = [2,4,6,8]
    fig , axs = plt.subplots(len(num_basis_list))
    i = 0
    for num_basis in num_basis_list:
        DMP_helper = DMP_trajectory_generator(num_basis=num_basis)
        weights = DMP_helper.learn_weights_from_raw_trajectory(q=q, dq=dq, ddq=ddq)
        out = DMP_helper.generate_dmp_trajectory()
        q_out = np.array(out['q'])
        
        for z in range(q_out.shape[-1]):
            axs[i].plot (out['time'], q_out[:,z], label=z)
        axs[i].legend()
        axs[i].set_title('num_basis {}'.format(num_basis))
        i += 1

    plt.show()