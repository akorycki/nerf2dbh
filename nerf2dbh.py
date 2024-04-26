import pclpy
import numpy as np
import datetime
import json
import yaml
import argparse
import sys
import treetool.seg_tree as seg_tree
import treetool.utils as utils
import treetool.tree_tool as tree_tool
import pandas as pd
from   scipy.optimize import linear_sum_assignment
import matplotlib.pyplot as plt

def main(pcd_file_path, dataparser_transforms_json_file_path, down_sample_rate, show_figs, is_metric):        
    print("         ,--.")
    print("       ,--.'|          ,-.----.       ,---,.              ,----,              ,---,        ,---,.       ,--.'| ")
    print("   ,--,:  : |          \    /  \    ,'  .' |            .'   .' \           .'  .' `\    ,'  .'  \   ,--,  | : ")
    print(",`--.'`|  ' :          ;   :    \ ,---.'   |          ,----,'    |        ,---.'     \ ,---.' .' |,---.'|  : ' ")
    print("|   :  :  | |          |   | .\ : |   |   .'          |    :  .  ;        |   |  .`\  ||   |  |: ||   | : _' | ")
    print(":   |   \ | :   ,---.  .   : |: | :   :  :            ;    |.'  /         :   : |  '  |:   :  :  /:   : |.'  | ")
    print("|   : '  '; |  /     \ |   |  \ : :   |  |-,          `----'/  ;          |   ' '  ;  ::   |    ; |   ' '  ; : ")
    print("'   ' ;.    ; /    /  ||   : .  / |   :  ;/|            /  ;  /           '   | ;  .  ||   :     \\'   |  .'. | ")
    print("|   | | \   |.    ' / |;   | |  \ |   |   .'           ;  /  /-,          |   | :  |  '|   |   . ||   | :  | ' ")
    print("'   : |  ; .''   ;   /||   | ;\  \\'   :  '            /  /  /.`|          '   : | /  ; '   :  '; |'   : |  : ; ")
    print("|   | '`--'  '   |  / |:   ' | \.'|   |  |          ./__;      :          |   | '` ,/  |   |  | ; |   | '  ,/  ")
    print("'   : |      |   :    |:   : :-'  |   :  \          |   :    .'           ;   :  .'    |   :   /  ;   : ;--'   ")
    print(";   |.'       \   \  / |   |.'    |   | ,'          ;   | .'              |   ,.'      |   | ,'   |   ,/       ")
    print("'---'          `----'  `---'      `----'            `---'                 '---'        `----'     '---'       \n\n")
    print("A NeRF-TreeTool bridge for neural tree diameter estimation\n")
    print("TreeTool: https://github.com/porteratzo/TreeTool (Montoya et al)")
    print("NeRF:     Representing Scenes as Neural Radiance Fields for View Synthesis (Mildenhall et al)\n")

    # Load point cloud data
    PointCloud = pclpy.pcl.PointCloud.PointXYZ()
    pclpy.pcl.io.loadPCDFile(pcd_file_path,PointCloud)
    print("[]Loaded feature cloud from " + pcd_file_path + " [" + str(round(PointCloud.xyz.size/down_sample_rate, 0))+ " points]")
    # Load world frame transform (eg NeRF->World)
    with open(dataparser_transforms_json_file_path, 'r') as file:
        json_data = json.load(file)
        # Extract the transform matrix and scale factor
        transform_matrix = np.array(json_data['transform'])
        scale_factor = json_data['scale']

    print("[]Loaded NeRF to world transformation from " + json_file_path)
    # Print the transform matrix and scale factor
    print("\ntransform Matrix:")
    for row in transform_matrix:
        # Convert each value in the row to a string and join them with a tab ('\t') or space for better readability
        print("\t".join(map(str, row)))
    if is_metric == False:
        print("scale Factor:", str(1/scale_factor) +"\n")
    else:
        print("scale Factor: 1.0\n")

    print("[]Starting TreeTool pipeline\n")
    # Full TreeTool pipeline (https://github.com/porteratzo/TreeTool) Montoya et al
    PointCloudV = seg_tree.voxelize(PointCloud.xyz,down_sample_rate) # subsample point cloud according to <down_sample_rate>
    PointCloud_scaled = seg_tree.transform_and_scale_point_cloud(json_data, is_metric, PointCloudV)
    if show_figs:
        utils.open3dpaint(PointCloud_scaled, reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool = tree_tool.treetool(PointCloud_scaled)
    My_treetool.step_1_remove_floor(set_initial_distance=0.5, set_max_distance=1.2)
    print('[]Segmented ground')
    if show_figs:
        utils.open3dpaint([My_treetool.non_ground_cloud,My_treetool.ground_cloud],reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool.step_2_normal_filtering(search_radius=0.08, verticality_threshold=0.1, curvature_threshold=0.3)
    if show_figs:
        utils.open3dpaint([My_treetool.non_ground_cloud.xyz, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.1, My_treetool.non_filtered_points.xyz + My_treetool.non_filtered_normals * 0.2], reduce_for_vis = False , voxel_size = 0.01)
        utils.open3dpaint([My_treetool.filtered_points.xyz, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.05, My_treetool.filtered_points.xyz + My_treetool.filtered_normals * 0.1], reduce_for_vis = False , voxel_size = 0.01)
    My_treetool.step_3_euclidean_clustering(tolerance=0.2, min_cluster_size=40, max_cluster_size=6000000)
    print('[]Clustered sub stems')
    if show_figs:
        utils.open3dpaint(My_treetool.cluster_list,reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool.step_4_group_stems(max_distance=1.0)
    print('[]Grouped stem segments')    
    if show_figs: 
        utils.open3dpaint(My_treetool.complete_Stems,reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool.step_5_get_ground_level_trees(lowstems_height=5, cutstems_height=5)
    if show_figs:
        utils.open3dpaint(My_treetool.low_stems,reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool.step_6_get_cylinder_tree_models(search_radius=0.1)
    print('[]Fitted cylinder tree models')
    if show_figs:
        utils.open3dpaint([i['tree'] for i in My_treetool.finalstems] + My_treetool.visualization_cylinders,reduce_for_vis = False  , voxel_size = 0.01)
    My_treetool.step_7_ellipse_fit()
    print('[]Fitted ellipse tree crossection models\n')
    if show_figs:
        utils.open3dpaint([i['tree'] for i in My_treetool.finalstems] + My_treetool.visualization_cylinders,reduce_for_vis = False  , voxel_size = 0.01)

    # get point cloud name
    pointcloud_name = 'your_cloud'
    last_slash_index = pcd_file_path.rfind('/')
    pcd_index = pcd_file_path.find('.pcd', last_slash_index)
    if pcd_index != -1 and last_slash_index != -1:
        pointcloud_name =  pcd_file_path[last_slash_index + 1:pcd_index]
    save_location = 'results/' + pointcloud_name + '.csv'
    My_treetool.save_results(save_location)
    print('[]Results saved to ' + save_location)

if __name__ == "__main__":
    # Create argument parser
    parser = argparse.ArgumentParser(description='A NeRF-TreeTool bridge for neural tree diameter estimation')
    parser.add_argument('config_yaml_filepath', type=str, help='path to config yaml')
    args = parser.parse_args()

    # Open and read the YAML file
    with open(str(args.config_yaml_filepath), 'r') as file:
        data = yaml.safe_load(file)

    # Extract the fields from the YAML data and store them in variables
    pcd_file_path = data.get('pcd_file_path', '')
    json_file_path = data.get('json_file_path', '')
    down_sample_rate = data.get('down_sample_rate', 0.0)
    is_metric = data.get('is_metric', False)
    show_figs = data.get('show_figs', False)

    # Parse the arguments
    args = parser.parse_args()
    
    # Call the main function with parsed arguments
    main(pcd_file_path, json_file_path, down_sample_rate, show_figs, is_metric)