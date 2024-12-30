import cfspy
import numpy as np
from pydrake.all import *
import numpy as np
import time
import ipdb
from support_functions import AddShape, RigidTransform2Array, show_robot,construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion

if __name__ == '__main__':
    # build the robot plant 
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    parser = Parser(plant)
    parser.package_map().Add("drake_project", "../")        

    directives = LoadModelDirectives("models/two_iiwa/two_robot.yaml")

    models = ProcessModelDirectives(directives, plant, parser)

    # construct the scene 
    block1 = AddShape(
        plant, Box(0.2, 0.05, 0.05), "block1", mass= 1, mu = 1,color=[1, 0, 0, 1]
    )
    
    plant.SetDefaultFreeBodyPose(
        plant.GetBodyByName("block1", block1),
        RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.5, 0.1]),
    )
   
    floor = AddShape(
        plant, Box(1.5, 3, 0.1), "floor", mass= 1, mu = 1,color=[0.835, 0.835, 0.835, 1]
    )
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("floor", floor),
        RigidTransform(RotationMatrix(),[0, 0.75, -0.05]),
    )
    # add frame for visulzation
    iiwa_attach_frame = dict()
    for i in range(2):
        iiwa_attach_frame[i] = plant.AddFrame(
        FixedOffsetFrame(
            f"iiwa_{i}_attach_frame",
            plant.GetFrameByName("iiwa_link_ee", plant.GetModelInstanceByName(f"iiwa_{i+1}")),
            RigidTransform(RollPitchYaw(0, 0, 0).ToRotationMatrix(),np.array([0.2,0,0])),
        )
    )
    # build plant and diagram
    plant.Finalize()
    ctrl = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
    builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port())
    params = MeshcatVisualizerParams()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    visualizer.StartRecording(True)
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    context = diagram.CreateDefaultContext()

    # user defined handover target points
    robot_num = 2
    object_num = 1 
    robot1_init = np.array([0,0,0,0,0,0,0])
    robot2_init = np.array([0,0,0,0,0,0,0])
    robot_init = np.concatenate((robot1_init,robot2_init))
    joint_label = {
        'robot_init':  np.concatenate((robot1_init,robot2_init)),
        'robot1_in_target1': np.concatenate((np.array([ 1.68429808, -0.52752681, -0.14421902,  1.81495976,  0.09874811, -0.799605  ,  3.05432619]),robot2_init)),
        'robot12_handover': np.array([ 1.57693503, -0.2936276 , -0.00756656, -1.38396305,  0.01131036,0.42412256,  0.00436817 , -1.45542816,-0.0833111 , -0.09918017, -1.1025703 , -0.03430604,  0.61549601, 0.00488511]),
        'robot2_in_target2': np.concatenate((robot1_init,np.array([1.70393825,0.52799513, -0.16757439, -1.81664391,  0.11246097,  0.79813863, 3.0470201 ]))),
    }
    
    # interpolate path, can be replaced with CFS
    num_points = 1000
    path = []
    path.append(np.linspace(joint_label['robot_init'], joint_label['robot1_in_target1'], num=num_points))
    path.append(np.linspace(joint_label['robot1_in_target1'], joint_label['robot12_handover'], num=num_points))
    path.append(np.linspace(joint_label['robot12_handover'], joint_label['robot2_in_target2'], num=num_points))
    
    # drake sim 
    object_init_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, -0.5, 0.1]))
    object_goal_pose = RigidTransform2Array(RigidTransform(RollPitchYaw(-np.pi/2,np.pi/2,0).ToRotationMatrix(),[0, 1.75, 0.1]))
    count = 0
    time_step = 0.1
    object_in_robot_index = 0
    robot_attach_pose = dict()
    robot_attach_pose[0] = object_init_pose
    robot_attach_pose[robot_num+1] = object_goal_pose
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    current_moving_object = 0
    visualizer.StartRecording()
    visualizer_context = visualizer.GetMyContextFromRoot(diagram_context)
    open_gripper = np.array([-0.06,0.06])
    close_gripper = np.array([-0.025,0.025])
            
    sim_time = 0
    for trajectory in path:
        sim_time = sim_time + 0
        for t in range(trajectory.shape[0]):
            sim_time = sim_time + 0.005
            diagram_context.SetTime(sim_time)   
                
            # calculate the robot end-effector position
            joint_position = np.concatenate((trajectory[t][0:7].reshape(7,1),open_gripper.reshape(2,1), trajectory[t][7:14].reshape(7,1),open_gripper.reshape(2,1),object_init_pose[:object_num*7]))
            plant.SetPositions(plant_context, joint_position)
            for i in range(robot_num):
                robot_attach_pose[i+1] = RigidTransform2Array(plant.CalcRelativeTransform(plant_context, plant.world_frame(), iiwa_attach_frame[i]))
            
            # calculate the robot gripper and object position
            if count == 0:  # reach to pick position 
                robot1_gripper_target = open_gripper.reshape(2,1)
                robot2_gripper_target = open_gripper.reshape(2,1)
                q_object_real = object_init_pose[:object_num*7]
            elif count == 1:    # reach to handover position 
                robot1_gripper_target = close_gripper.reshape(2,1)
                robot2_gripper_target = open_gripper.reshape(2,1)
                q_object_real = robot_attach_pose[1]
            elif count == 2:    # reach to drop position 
                robot1_gripper_target = open_gripper.reshape(2,1)
                robot2_gripper_target = close_gripper.reshape(2,1)
                q_object_real = robot_attach_pose[2]
                
            target_position = np.concatenate((trajectory[t][0:7].reshape(7,1),robot1_gripper_target, trajectory[t][7:14].reshape(7,1),robot2_gripper_target,q_object_real))
            plant.SetPositions(plant_context, target_position)
            visualizer.ForcedPublish(visualizer_context)
        count = count + 1
    visualizer.StopRecording()
    visualizer.PublishRecording() 
    
    ipdb.set_trace()