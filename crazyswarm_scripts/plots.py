import csv
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go
import math
#
from MPC_take2.quad_index import *

#def uav_plot():
def main():
    file_path = 'MPC_take2/data/node_data.csv'

    #labels = ('px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz')
    #['cf_px', 'cf_py', 'cf_pz','cf_gx', 'cf_gy', 'cf_gz', 'h_px', 'h_py', 'h_pz', 'h_gx', 'h_gy', 'hg_z', 'tiltx', 'global time'])


    # Read data from CSV file
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        header = next(csv_reader)  # Read the header
        data = [row for row in csv_reader]


    # Extract columns 
    x_d = [float(row[kPosX]) for row in data]     #0
    y_d = [float(row[kPosY]) for row in data]     #1
    z_d = [float(row[kPosZ]) for row in data]     #2



    x_dg = [float(row[3]) for row in data]
    y_dg = [float(row[4]) for row in data]
    z_dg = [float(row[5]) for row in data]

    x_b=[float(row[6]) for row in data]
    y_b=[float(row[7]) for row in data]
    z_b=[float(row[8]) for row in data]

    x_bg = [float(row[9]) for row in data]  #was 20
    y_bg = [float(row[10]) for row in data]
    z_bg = [float(row[11]) for row in data]
    # z_e = [float(row[22]) for row in data]

    # phi_boat = [float(row[23]) for row in data]
    # phiSqrd_boat = [float(row[24]) for row in data]
    t_s = [float(row[13]) for row in data]  #was 17
    tilt = [float(row[12]) for row in data]


    accz = [float(row[14]) for row in data]


    #################### Heat Map #############################
    # Define the function

        ####################
        ##### Plotting #####
        ####################
    #plt.axhline(y = 10.0, color = 'r', linestyle = '--') 
    plt.plot(t_s,x_d, label=f'Drone Position')
    plt.plot(t_s,x_dg, label=f'Drone Goal')
    plt.plot(t_s,x_b, label=f'Boat Position')
    plt.plot(t_s,x_bg,label=f'Boat Goal')
    #plt.plot(t_s, accz, label='Acc Z')
    plt.xlabel('Time(s)')
    #plt.ylabel(header[kPosX])
    #plt.title(f'UAV - Comparison of Actual and Desired X-Coordinate')
    plt.legend()
    plt.show()



#hi jess

#how are you today

#hi jess

#how are you today

#hi jess

#how are you today

#hi jess

#how are you today

#hi jess

#how are you today

#hi jess

#how are you today

#hi jess

#how are you today



    # # plt.axhline(y = 10.0, color = 'r', linestyle = '--') 
    # plt.plot(y_g, label=f'Y_g_uav')
    # plt.plot(y, label=f'Y')
    # plt.xlabel('Step')
    # plt.ylabel(header[kPosY])
    # plt.title(f'Plot of {header[kPosY]}')
    # plt.legend()
    # plt.show()
    #

    # Z error plot
    # plt.xlabel('Time')
    # plt.ylabel('e_g,z')
    # plt.title('Error of UAV Z-goal and USV')
    # plt.scatter(t_s, z_e, color='red', marker='.', label='UAV State')
    # plt.legend()
    # plt.show()


    # plt.axhline(y = 0.0, color = 'r', linestyle = '--') 
    # plt.plot(z_g, label=f'Drone goal')
    # plt.plot(z, label=f'Actual Drone z-coordinate')
    # plt.plot(z_b, label=f'Boat goal')
    # plt.xlabel('Step')
    # plt.ylabel(header[kPosZ])
    # plt.title(f'UAV - Comparison of Actual and Desired Z-Coordinate')
    # plt.legend()
    # plt.show()


    #     ###############################
    #     ##### Create 3D animation #####
    #     ###############################
    # fig = go.Figure(data=[go.Scatter3d(x=[], y=[], z=[], mode="markers",marker=dict(color="red", size=4))])
        
    # fig.update_layout(
    #         scene = dict(
    #         xaxis=dict(range=[min(x), max(x)], autorange=False),
    #         yaxis=dict(range=[min(y), max(y)], autorange=False),
    #         zaxis=dict(range=[min(z), max(z)], autorange=False),
    #         ))


    # frames = [go.Frame(data= [go.Scatter3d(
    #             x=x[:k+1], 
    #             y=y[:k+1],
    #             z=z[:k+1])],
                    
    #             traces= [0],
    #             name=f'frame{k}'      
    #         )for k  in  range(len(x)-1)]

    # fig.update(frames=frames)


    # fig.update_layout(updatemenus=[dict(type="buttons",
    #     buttons=[dict(label="Play",
    #                     method="animate",
    #     args=[None, dict(frame=dict(redraw=True,fromcurrent=True, mode='immediate'))      ])])])


    # fig.show()

def z_function(x, t):
    return (-math.pi/100.0*(x-40) * np.cos(math.pi/5.0*(t + 2*x)))**2


if __name__ == "__main__":
    main()