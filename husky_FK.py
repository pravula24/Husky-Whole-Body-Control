#!/usr/bin/env python
# coding: utf-8

import pinocchio as pin
import numpy as np
import sys
import os
os.environ['ROS_PACKAGE_PATH'] = "C:/Users/Prath/Downloads/New_Simscape_Assem"
import time

from pinocchio.visualize import MeshcatVisualizer


# Load the URDF.
urdf_model_path = "C:/Users/Prath/Downloads/New_Simscape_Assem/urdf/New_Simscape_Assem.urdf"

mesh_dir = "C:/Users/Prath/Downloads/New_Simscape_Assem"

#building model from URDF
package_dirs = {
    "New_Simscape_Assem": "C:/Users/Prath/Downloads/New_Simscape_Assem"
}

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, root_joint=pin.JointModelFreeFlyer()
)
   
# Neutral configuration
q_neutral = pin.neutral(model)

#### visualization settings
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)	
viz.loadViewerModel()
viz.viewer["/Cameras/default"].set_property("position", [0, 1, 0]) # Set the camera position and orientation 


# Forward Kinematics
data = model.createData()
   


# Example foot link IDs
foot_link_ids = {
    "FL": model.getFrameId("Foot_FL"),
    "FR": model.getFrameId("Foot_FR"),
    "BL": model.getFrameId("Foot_BL"),
    "BR": model.getFrameId("Foot_BR"),
}

# Function to compute forward kinematics
def compute_foot_positions(joint_angles):
    """
    Compute foot positions in body and world frames.
    :param joint_angles: Joint configuration (np.array or list of joint angles)
    :return: Dictionary with foot positions in body and world frames
    """
    # Update the joint configuration
    q = q_neutral.copy()
    q[:len(joint_angles)] = joint_angles  # Update the given joint angles

    # Normalize the configuration
    q_normalized = pin.normalize(model, q)

    # Perform forward kinematics
    pin.forwardKinematics(model, data, q_normalized)
    pin.updateFramePlacements(model, data)

    # Calculate foot positions
    foot_positions = {}
    for foot, frame_id in foot_link_ids.items():
        world_position = data.oMf[frame_id].translation
        body_position = model.frames[frame_id].placement.actInv(world_position)  #check this out again
        foot_positions[foot] = {
            "world": world_position,
            "body": body_position,
        }

    return foot_positions, q_normalized


# Function to set joint angles for each leg and compute forward kinematics
def set_leg_angles(FR_angles, FL_angles, BL_angles, BR_angles):
    """Set joint angles for all four legs."""
    q = q_neutral.copy()  # Start with neutral configuration

    # Update joint angles for each leg (replace these indices with your robot's joint indices)
    # LF (Left Front leg)
   
    # q[model.getJointId("hip_FL")] = FL_angles[0]  #13
    # print(model.getJointId("hip_FL"))
    # q[model.getJointId("thigh_FL")] = FL_angles[1]  #14
    # print(model.getJointId("thigh_FL"))
    # q[model.getJointId("knee_FL")] = FL_angles[2] #16
    # print(model.getJointId("knee_FL"))
    # q[model.getJointId("controlRod_FL")] = FL_angles[2]  #15
    # print(model.getJointId("controlRod_FL"))
    # q[model.getJointId("ankle_FL")] = -FL_angles[2]  #15
    # print(model.getJointId("ankle_FL"))
    
    q[22] = FL_angles[0]  #13

    q[23] = FL_angles[1]  #14
   
    q[25] = FL_angles[2] #16

    q[24] = FL_angles[2]  #15
   
    q[26] = -FL_angles[2]  #15
    


    q[model.getJointId("hip_FR")] = FR_angles[0]  #13
    print(model.getJointId("hip_FR"))
    q[model.getJointId("thigh_FR")+1] = FR_angles[1]  #14
    print(model.getJointId("thigh_FR"))
    q[model.getJointId("knee_FR")+1] = FR_angles[2] #16
    q[model.getJointId("controlRod_FR")] = FR_angles[2]  #15
    q[model.getJointId("ankle_FR")+1] = -FR_angles[2]  #15


    q[model.getJointId("hip_BL")] = BL_angles[0]  #13
    print(model.getJointId("hip_BL"))
    q[model.getJointId("thigh_BL")] = BL_angles[1]  #14
    q[model.getJointId("knee_BL")] = BL_angles[2] #16
    q[model.getJointId("controlRod_BL")] = BL_angles[2]  #15
    q[model.getJointId("ankle_BL")] = -BL_angles[2]  #15


    q[model.getJointId("hip_BR")] = BR_angles[0]  #13
    print(model.getJointId("hip_BR"))
    q[model.getJointId("thigh_BR")] = BR_angles[1]  #14
    q[model.getJointId("knee_BR")] = BR_angles[2] #16
    q[model.getJointId("controlRod_BR")] = BR_angles[2]  #15
    q[model.getJointId("ankle_BR")] = -BR_angles[2]  #15


    # # RH (Right Hind leg)
    # q[model.getJointId("RH_HFE")] = RH_angles[0]   #18
    # q[model.getJointId("RH_KFE")] = RH_angles[1]  #19
    # q[model.getJointId("br_control_rod")] = RH_angles[1] #21
    # q[model.getJointId("RH_ankle")] = RH_angles[2] #20
	
    
    return q
    
def visualize_motion():
    """
    Visualize the robot's motion in Meshcat using custom joint angles.
    """
    # Case 1: Move legs based on desired joint angles
    # Example: Move each leg to specific joint angles in radians
    FR_angles = [0,0,0]  
    FL_angles = [0,0,0] 
    BL_angles = [0,0, 0] 
    BR_angles = [0,0, 0]  

    print(q_neutral)
    print("Joint Names and IDs:")
    for joint_id, joint_name in enumerate(model.names):
        print(f"Joint ID: {joint_id}, Joint Name: {joint_name}")
    # Get the updated configuration
    q = set_leg_angles(FR_angles, FL_angles, BL_angles, BR_angles)

    # Normalize the configuration
    q_normalized = pin.normalize(model, q)

    # Compute and print foot positions
    foot_positions, _ = compute_foot_positions(q_normalized)
    for foot, positions in foot_positions.items():
        print(f"{foot} Foot - World Frame: {positions['world']}, Body Frame: {positions['body']}")
        
    
    ## Case 2: 
    ## Comment above  and Uncomment this to give a specific joint configuration ##
    # Example joint configuration (replace with your desired angles)
    # q =  [ 0.      ,    0.       ,   0.      ,    0.        ,  0.        ,  0.,
    #         1.     ,     0.04586771 , 0.0062084  , 0.01084412 , 0.01084412, -0.01084412,
    #         -0.01920081 ,-0.00163682 , 0.00444871,  0.00444871 , 0.00444871 , 0.04193863,
    #         0.99998286 ,-0.00585445 , 0.01013431 , 0.01013431, -0.01013431,  0.04326467,
    #         0.01000075, -0.0156718  ,-0.0156718  ,-0.0156718 ]
    # q = np.array(q)
    # q = pin.normalize(model, q)

    # Visualize in Mes
    viz.display(q)
    # viz.display(q_neutral)
    time.sleep(3)  # Pause to see the motion



if __name__ == "__main__":
    visualize_motion()

