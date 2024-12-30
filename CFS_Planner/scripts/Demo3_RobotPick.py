import cfspy
import numpy as np
from pydrake.all import *
import numpy as np
import time
import ipdb
import graphviz
from support_functions import AddShape, RigidTransform2Array, show_robot,construct_labeled_convex_region, construct_connected_convex_region_RRT,RefineRegion

def MakeCommandTrajectory():
    T = 5.0
    open_gripper = np.array([-0.06,0.06])
    close_gripper = np.array([0,0])
    joint_label = {
        'robot_init':  np.concatenate((robot1_init,open_gripper,robot2_init,open_gripper)),
        'robot1_in_target1': np.concatenate((np.array([ 1.68429808, -0.52752681, -0.14421902,  1.81495976,  0.09874811, -0.799605  ,  3.05432619]),open_gripper,robot2_init,open_gripper)),
        'robot1_in_target1_gripper': np.concatenate((np.array([ 1.68429808, -0.52752681, -0.14421902,  1.81495976,  0.09874811, -0.799605  ,  3.05432619]),close_gripper,robot2_init,open_gripper)),
        'robot12_handover': np.concatenate((np.array([ 1.57693503, -0.2936276 , -0.00756656, -1.38396305,  0.01131036,0.42412256,  0.00436817]),close_gripper,robot2_init,open_gripper)),
        # 'robot12_handover_gripper': np.concatenate((np.array([ 1.57693503, -0.2936276 , -0.00756656, -1.38396305,  0.01131036,0.42412256,  0.00436817]),open_gripper,np.array([-1.45542816,-0.0833111 , -0.09918017, -1.1025703 , -0.03430604,  0.61549601, 0.00488511]),close_gripper)),
        # 'robot2_in_target2': np.concatenate((robot1_init,open_gripper,np.array([1.70393825,0.52799513, -0.16757439, -1.81664391,  0.11246097,  0.79813863, 3.0470201 ]),close_gripper)),
    }
    Target = np.vstack((joint_label['robot_init'], joint_label['robot1_in_target1'],joint_label['robot1_in_target1_gripper'],joint_label['robot12_handover'])).T
    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [0, T, 2 * T, 3* T],               # times
        Target,
    )

    return traj_wsg_command

if __name__ == '__main__':
    time_step = 1e-5
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
        plant, Box(0.2, 0.05, 0.05), "block1", mass= 0.001, mu = 5,color=[1, 0, 0, 1]
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
    iiwa_1 = plant.GetModelInstanceByName("iiwa_1")
    iiwa_2 = plant.GetModelInstanceByName("iiwa_2")

    wsg_1 = plant.GetModelInstanceByName("wsg_1")
    wsg_2 = plant.GetModelInstanceByName("wsg_2")

    # build plant and diagram
    plant.Finalize()
    params = MeshcatVisualizerParams()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    visualizer.StartRecording(True)


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
    
    robot_dof = (7+2)*2
    # print(plant.num_positions())
    kp = [10] * robot_dof
    ki = [1] * robot_dof
    kd = [10] * robot_dof

    # Make the plant for the iiwa controller to use.
    controller_plant = MultibodyPlant(time_step=time_step)
    parser = Parser(controller_plant)
    parser.package_map().Add("drake_project", "../")        

    directives = LoadModelDirectives("models/two_iiwa/two_robot.yaml")

    models = ProcessModelDirectives(directives, plant, parser)

    controller_plant.Finalize()

    iiwa_controller = builder.AddSystem(InverseDynamicsController(controller_plant, kp, ki, kd, False))
    iiwa_controller.set_name("iiwa_controller")
    
    iiwa_1_state_demultiplexer = builder.AddSystem(Demultiplexer([7, 7]))
    iiwa_1_state_demultiplexer.set_name("state_dultiplexer1")

    wsg_1_state_demultiplexer = builder.AddSystem(Demultiplexer([2, 2]))
    wsg_1_state_demultiplexer.set_name("state_dultiplexer2")

    iiwa_2_state_demultiplexer = builder.AddSystem(Demultiplexer([7, 7]))
    iiwa_2_state_demultiplexer.set_name("state_dultiplexer3")

    wsg_2_state_demultiplexer = builder.AddSystem(Demultiplexer([2, 2]))
    wsg_2_state_demultiplexer.set_name("state_dultiplexer4")

    builder.Connect(
        plant.get_state_output_port(iiwa_1),
        iiwa_1_state_demultiplexer.get_input_port(),
    )

    builder.Connect(
        plant.get_state_output_port(wsg_1),
        wsg_1_state_demultiplexer.get_input_port(),
    )

    builder.Connect(
        plant.get_state_output_port(iiwa_2),
        iiwa_2_state_demultiplexer.get_input_port(),
    )

    builder.Connect(
        plant.get_state_output_port(wsg_2),
        wsg_2_state_demultiplexer.get_input_port(),
    )

    state_multiplexer = builder.AddSystem(Multiplexer([7, 2, 7, 2, 7, 2, 7, 2]))

    builder.Connect(
        iiwa_1_state_demultiplexer.get_output_port(0),
        state_multiplexer.get_input_port(0),
    )
    builder.Connect(
        wsg_1_state_demultiplexer.get_output_port(0),
        state_multiplexer.get_input_port(1),
    )
    builder.Connect(
        iiwa_2_state_demultiplexer.get_output_port(0),
        state_multiplexer.get_input_port(2),
    )

    builder.Connect(
        wsg_2_state_demultiplexer.get_output_port(0),
        state_multiplexer.get_input_port(3),
    )

    builder.Connect(
        iiwa_1_state_demultiplexer.get_output_port(1),
        state_multiplexer.get_input_port(4),
    )
    builder.Connect(
        wsg_1_state_demultiplexer.get_output_port(1),
        state_multiplexer.get_input_port(5),
    )
    builder.Connect(
        iiwa_2_state_demultiplexer.get_output_port(1),
        state_multiplexer.get_input_port(6),
    )

    builder.Connect(
        wsg_2_state_demultiplexer.get_output_port(1),
        state_multiplexer.get_input_port(7),
    )

    builder.Connect(
        state_multiplexer.get_output_port(),
        iiwa_controller.get_input_port_estimated_state(),
    )

    state_demultiplexer = builder.AddSystem(Demultiplexer([7, 2, 7, 2]))
    state_demultiplexer.set_name("state_dultiplexer")
    builder.Connect(
        iiwa_controller.get_output_port_control(), state_demultiplexer.get_input_port()
    )
    builder.Connect(
        state_demultiplexer.get_output_port(0), plant.get_actuation_input_port(iiwa_1)
    )
    builder.Connect(
        state_demultiplexer.get_output_port(1), plant.get_actuation_input_port(wsg_1)
    )
    builder.Connect(
        state_demultiplexer.get_output_port(2), plant.get_actuation_input_port(iiwa_2)
    )
    builder.Connect(
        state_demultiplexer.get_output_port(3), plant.get_actuation_input_port(wsg_2)
    )

    # Add trajectory generator
    gripper_traj_command = MakeCommandTrajectory()

    total_path_block = builder.AddSystem(TrajectorySource(gripper_traj_command))
    total_path_block.set_name("total_path_with_gripper")
    
    # Add discrete derivative to command velocities.
    desired_state_from_position = builder.AddSystem(
        StateInterpolatorWithDiscreteDerivative(
            (7 + 2) * 2,
            time_step,
            suppress_initial_transient=True,
        )
    )
    builder.Connect(
        total_path_block.get_output_port(),
        desired_state_from_position.get_input_port(),
    )
    builder.Connect(
        desired_state_from_position.get_output_port(),
        iiwa_controller.get_input_port_desired_state(),
    )

    diagram = builder.Build()

    graph = graphviz.Source(diagram.GetGraphvizString())
    graph.render(filename='svg', format='png', cleanup=True)

    context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, context)

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)
    meshcat.StartRecording(set_visualizations_while_recording=True)
    simulator.AdvanceTo(15.0)
    meshcat.PublishRecording()
    print("simulation done")
    ipdb.set_trace()
 