'''
Created on Aug 5, 2019

@author: modys
'''

if __name__ == '__main__':
    pass

import sys
import numpy as np
import matplotlib.pyplot as plt
from dynamicModel import Vehicle

sys.path.append('../')
sys.path.append('../Algos')
from pid import PID

'''
debug
'''
debug = False
'''
Set the simulation time
'''     
cyclic_time = 0.01;
time_end = 600;

'''
Init & Configure vehicle model
'''
model = Vehicle(time_end,cyclic_time);
model.reset();
model.setEngineCfg();
model.setVehicleCfg();
model.setFrictionCoff();
model.setTireForces();

'''
Init & Configure PID controller
'''
controller = PID(Kp=2.0, Ki=0.3, Kd=100.0,windupVal=80);
controller.output_limits = (0.0,100.0);

'''
Data Saving for results plotting
'''
t_data = np.arange(0,time_end,cyclic_time);
set_point_data = np.zeros_like(t_data);
intgeral_data = np.zeros_like(t_data);
derivative_data = np.zeros_like(t_data);
output_data = np.zeros_like(t_data);
proportional_data = np.zeros_like(t_data);


# throttle value Range(0.0 --> 1.0)
throttle = 0.0
 
# incline angle (in radians)
alpha = 0

# init set_point
set_point = 5.0;

'''
control loop
'''

for i in range(model.getDataLength()):
    time = i*cyclic_time
    
    
    if(time>0 and time <200):
        set_point = 10;
         
    if(time>200 and time<400):
        set_point = 30;
        
    if(time>400):
        set_point = 20;
            
    throttle = controller.update(set_point,model.getVelocity(),cyclic_time)/100.0;
    model.step(throttle, alpha,set_point ,i);
    
    proportional_data[i] = controller.get_proportioanlPart();
    derivative_data[i] = controller.get_derivativePart();
    intgeral_data[i] = controller.get_intgeralPart();
    output_data[i] = controller.get_sumOutput();
    set_point_data[i] = set_point;
    
model.plotResults();

if debug is True:
    plt.figure(1)
    plt.title('intgeral')
    plt.plot(t_data, intgeral_data)
    plt.legend()
    plt.grid()
    
    
    plt.figure(2)
    plt.title('derivative')
    plt.plot(t_data, derivative_data)
    plt.legend()
    plt.grid()
    
    
    plt.figure(3)
    plt.title('error')
    plt.plot(t_data, proportional_data)
    plt.legend()
    plt.grid()
    
    
    plt.figure(4)
    plt.title('output')
    plt.plot(t_data, output_data)
    plt.legend()
    plt.grid()
    plt.show()
    