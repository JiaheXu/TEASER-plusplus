import numpy as np
import open3d as o3d
import os
import pickle

import teaserpp_python
from mpl_toolkits.mplot3d import Axes3D
# Plot
import matplotlib.pyplot as plt

def load_object_goals(obj_name):
    # Load the pcds
    file_path = obj_name + ".pkl"
    with open(file_path, 'rb') as f:
        content = pickle.load(f)
        pcds_np = content['pcds']
        imgs = content['imgs']
    
    # Convert to open3d
    pcds = []
    # print("pcds_np: ", pcds_np)
    for pcd_np in pcds_np:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_np)
        pcds.append(pcd)
    
    return pcds_np, imgs

def find_transform(src, dst):
    
    # Populating the parameters
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = 0.01
    solver_params.estimate_scaling = True
    solver_params.rotation_estimation_algorithm = teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 100
    solver_params.rotation_cost_threshold = 1e-12
    print("Parameters are:", solver_params)

    solver = teaserpp_python.RobustRegistrationSolver(solver_params)

    solver.solve(src, dst)
    solution = solver.getSolution()
    return solution

if __name__ == '__main__':
    obj_name = "brown_car"
    # obj_name = "color_block2"
    # obj_name = "color_block3"
    # Check the results
    pcds_np, imgs = load_object_goals(obj_name)
    colors = ['tab:red' ,'tab:blue', 'tab:orange', 'tab:green']
    
    pcd_np = []
    
    for pcd in pcds_np:
        pcd_np.append( np.transpose(pcd) )

    solution = find_transform(pcd_np[0], pcd_np[1])

    print("solution is: ", solution)
    print("rotation: ", solution.rotation)
    
    results = []
    results.append(pcd_np[1])

    trans = np.transpose(solution.translation)
    trans = trans.reshape(3,1)
    pcd = solution.rotation @ pcd_np[0]+ trans
    results.append( pcd )


    print("solution is: ", solution)



    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")
    for i in range( 2 ):
        pcds = results[i]
        xs = pcds[0, :]
        ys = pcds[1, :]
        zs = pcds[2, :]
        ax.scatter(xs, ys, zs, c = colors[i])

    plt.show()
 

#  translation=
# -0.012568
#  0.205561
# 0.0112908
# rotation=
#   0.319831   -0.89319   0.316099
#  0.0676912   -0.31123  -0.947921
#   0.945053   0.324572 -0.0390798
