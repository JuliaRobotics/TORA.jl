# Adding a New Robot

This tutorial explains how to use TORA.jl with a robot that has not yet been added to the package.

As an example, I will demonstrate how to add the [Kinova Gen3 lite](https://www.kinovarobotics.com/en/products/gen3-lite-robot) robot.

!!! tip
    If you are adding a widely available robot, consider creating a [fork](https://guides.github.com/activities/forking/) of TORA.jl and following this guide such that you can submit a [Pull Request](https://guides.github.com/activities/forking/#making-a-pull-request) once you are done.

## Add the Robot Submodule

Usually, widely-used robots have existing GitHub repositories associated with them. For the *Kinova Gen3 lite*, that repository is [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex).

To add the repository as a submodule, I will run
```
git submodule add https://github.com/Kinovarobotics/ros_kortex
```
inside the root of the TORA.jl package.

## Prepare the Robot URDF

The next step is to prepare the URDF model of the robot, which we will load later using [RigidBodyDynamics.jl](https://github.com/JuliaRobotics/RigidBodyDynamics.jl).

After a bit of searching, I found the `.xacro` file of the robot I want under `TORA.jl/ros_kortex/kortex_description/robots/gen3_lite_gen3_lite_2f.xacro`.

Inside a [catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), I will convert the `.xacro` file into a `.urdf` by running
```
rosrun xacro xacro -o gen3_lite_gen3_lite_2f.urdf gen3_lite_gen3_lite_2f.xacro
```

Afterwards, I'll just move the generated `.urdf` file to the `TORA.jl/robots` folder.

Now I need to open the `.urdf` with a text editor, and look for the end-effector frame. Scrolling down, I see:

```xml
<!-- Tool frame used by the arm -->
<link name="tool_frame"/>
```

I will note down the name of this frame, as I will need it soon.

!!! note
    The default `.urdf` of other robots may not come with a tool frame. In such cases, you can add one manually. Look at other existing `.urdf` files for examples in [TORA.jl/robots](https://github.com/ferrolho/TORA.jl/tree/main/robots).

As I scroll through the `.urdf`, I notice that this robot has two actuated fingers:

```xml
(...)
<joint name="right_finger_bottom_joint" type="revolute">
(...)
<joint name="right_finger_tip_joint" type="revolute">
(...)
<joint name="left_finger_bottom_joint" type="revolute">
(...)
<joint name="left_finger_tip_joint" type="revolute">
(...)
```

I am not very interested in planning a trajectory for each of these joints with TORA.jl, so I will manually edit the `.urdf` and change the type of these joints from `"revolute"` to `"fixed"`.

## Write the Create Method

The last thing I need to do is write a method wrapping the [`Robot`](@ref) constructor in `src/robot.jl`.

```julia
function create_robot_kinova_gen3_lite(vis::Visualizer)
    package_path = joinpath(@__DIR__, "..", "ros_kortex")
    urdfpath = joinpath(@__DIR__, "..", "robots", "gen3_lite_gen3_lite_2f.urdf")

    mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
    frame_ee = default_frame(findbody(mechanism, "tool_frame"))
    remove_fixed_tree_joints!(mechanism)

    urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
    mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
    # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

    Robot(urdfpath, mechanism, frame_ee, mvis)
end
```

As a convention, the name of this method should start with `create_robot_` followed by the name of the robot.
The string `package_path` should point to the submodule of the robot, which contains the meshes needed to render the robot in the visualizer.
The `urdfpath` string should point to the `.urdf` file of the robot.
Finally, `frame_ee` should be assigned to the tool frame of the robot, which in this case is `"tool_frame"`—the name I had noted down before.

## Test the New Robot

The robot is now ready to be used with TORA.jl. To test it out, I will try to run the [Tutorial](@ref) notebook with the new robot. Instead of calling
```julia
robot = TORA.create_robot_kuka_iiwa_14(vis)
```
I will call
```julia
robot = TORA.create_robot_kinova_gen3_lite(vis)
```

Running the rest of the tutorial notebook, I get:

```@raw html
<video src="../assets/videos/kinova_gen3_lite.mp4" autoplay loop muted width="100%" style="margin-bottom: 1em;">Your browser does not support the video tag.</video>
```

!!! note
    The *Kinova Gen3 lite* is smaller than the *KUKA LBR iiwa 14* used in the [Tutorial](@ref).
    Therefore, the circle that the robot is tracing in the video above is lower, and the position of the end-effector in task-space is given by
    ```julia
    pos = [0.5, 0.2 * cos(θ), 0.5 + 0.2 * sin(θ)]
    ```

!!! note
    To make the video above loop seamlessly, I also fixed the initial and final joint positions to the same configuration, using the [`fix_joint_positions!`](@ref) method.
