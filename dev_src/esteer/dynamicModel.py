'''
Created on Aug 6, 2019

@author: modys
'''
import numpy as np
import matplotlib.pyplot as plt

class Vehicle(object):
    '''
    classdocs
    '''


    def __init__(self,time_end,cyclic_time):
        '''
        Constructor
        '''
    
        #Throttle to engine torque
        self.a_0_m_s_2 = 0;
        self.a_1_m_s_3 = 0;
        self.a_2_m_s_4 = 0;
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.0;
        self.r_e_m = 0.0;
        self.J_e_kg_m_2 = 0;
        self.m_kg = 0;
        self.g_m_s_2 = 0;
        
        # Aerodynamic and friction coefficients
        self.c_a_kg_m = 0.0;
        self.c_r1_kg_s = 0.0;
        
        # Tire force 
        self.c_N = 0;
        self.F_max_N = 0;
        
        # State variables
        self.x_m = 0;
        self.v_m_s = 0;
        self.a_m_s_2 = 0;
        self.w_e_rad_s = 0.1;
        self.w_e_dot_rad_s_2 = 0;
        
        # Sample time
        self.sample_time_s = cyclic_time;
        
        # Data Saving for results plotting
        self.t_data = np.arange(0,time_end,cyclic_time);
        self.v_data = np.zeros_like(self.t_data);
        self.a_data = np.zeros_like(self.t_data);
        self.throttle_data = np.zeros_like(self.t_data);
        self.alpha_data = np.zeros_like(self.t_data);
        
        self.set_point = np.zeros_like(self.t_data);
        
    def setEngineCfg(self,a0_m_s_2= 400, a1_m_s_3= 0.1, a2_m_s_4= -0.0002):
        #Throttle to engine torque
        self.a_0_m_s_2 = a0_m_s_2;
        self.a_1_m_s_3 = a1_m_s_3;
        self.a_2_m_s_4 = a2_m_s_4;
        
    def setVehicleCfg(self,gearRatio=0.35, effective_radious_m= 0.3, intertia_kg_m_2=10, mass_kg= 2000, g_m_s_2= 9.81):
        # Gear ratio, effective radius, mass + inertia
        self.GR = gearRatio;
        self.r_e_m = effective_radious_m;
        self.J_e_kg_m_2 = intertia_kg_m_2;
        self.m_kg = mass_kg;
        self.g_m_s_2 = g_m_s_2;
        
    def setFrictionCoff(self,aero_kg_m= 1.36,roll_kg_s = 0.01):
        # Aerodynamic and friction coefficients
        self.c_a_kg_m = aero_kg_m;
        self.c_r1_kg_s = roll_kg_s;
        
    def setTireForces(self,c_N= 10000,Fmax_N= 10000):
        # Tire force 
        self.c_N = c_N;
        self.F_max_N = Fmax_N;

    def reset(self):
        # reset state variables
        self.x_m = 0;
        self.v_m_s = 5;
        self.a_m_s_2 = 0;
        self.w_e_rad_s = 100;
        self.w_e_dot_rad_s_2 = 0 ;
        
    def getDataLength(self):
        return self.t_data.shape[0];   
        
    def step(self, throttle, alpha,set_point,i):
        # ==================================
        #  Implement vehicle model here
        # ==================================
        w_w = self.GR * self.w_e_rad_s;
        s = (w_w * self.r_e_m - self.v_m_s) / self.v_m_s;
        
        if abs(s) < 1:
            F_x = self.c_N * s;
        else:
            F_x = self.F_max_N;
            
        F_aero = self.c_a_kg_m * (self.v_m_s ** 2);
        R_x = self.c_r1_kg_s * self.v_m_s;
        F_g = self.m_kg * self.g_m_s_2 * np.sin(alpha);
        F_load = R_x + F_aero + F_g;
        
        self.a_m_s_2 = (F_x - F_load) / self.m_kg;
        self.v_m_s += self.a_m_s_2 * self.sample_time_s;
        self.x_m += self.v_m_s * self.sample_time_s;
        T_e = throttle * (self.a_0_m_s_2 + self.a_1_m_s_3 * self.w_e_rad_s + self.a_2_m_s_4 * self.w_e_rad_s ** 2);
        self.w_e_dot_rad_s_2 = (T_e - self.GR * self.r_e_m * F_load) / self.J_e_kg_m_2;
        self.w_e_rad_s += self.w_e_dot_rad_s_2 * self.sample_time_s;
        
        # ==================================
        #  Record data for plotting results
        # ==================================        
        self.v_data[i] = self.v_m_s;
        self.a_data[i] = self.a_m_s_2;
        self.throttle_data[i] = throttle;
        self.alpha_data[i] = alpha;
        self.set_point[i] = set_point;
        
    def getVelocity(self):
        return self.v_m_s;
        
    def plotResults(self,debug=False):
        plt.figure(1)
        plt.title('Velocity')
        plt.plot(self.t_data, self.v_data,'b')
        plt.plot(self.t_data, self.set_point,'r')
        plt.legend()
        plt.grid()
        
        plt.figure(2)
        plt.title('Throttle')
        plt.plot(self.t_data, self.throttle_data)
        plt.legend()
        plt.grid()
        
        if(debug is True):
            plt.figure(3)
            plt.title('Acceleration')
            plt.plot(self.t_data, self.a_data)
            plt.legend()
            plt.grid()
            
            
            
            plt.figure(4)
            plt.title('alpha')
            plt.plot(self.t_data, self.alpha_data)
            plt.legend()
            plt.grid()
        
        plt.show()
        