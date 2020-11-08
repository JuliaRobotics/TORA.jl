# Tutorial

!!! info
    A [Jupyter](https://jupyter.org/) notebook of this tutorial is available [here](https://github.com/ferrolho/TORA.jl/blob/main/notebooks/Tutorial.ipynb).

## Setup

We are going to use TORA.jl as well as other packages to visualise and handle rigid-body mechanisms.

```julia
using TORA
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
```

```@setup example_1
using TORA
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

vis = Visualizer()
``` 

Try to call `TORA.greet()`. It should simply print "Hello World!"
```@example example_1
TORA.greet()
```

Next, we want to create a 3D viewer using [MeshCat.jl](https://github.com/rdeits/MeshCat.jl)
```julia
vis = Visualizer()
```

Let's open the visualizer (in a separate browser tab)
```julia
open(vis)
```

It should look like this:

![Empty viewer](../assets/screenshots/tutorial_01.png)

## Loading a Robot

After starting the viewer, we are now ready to load a robot. Let's load a [KUKA LBR iiwa 14](https://www.kuka.com/en-gb/products/robotics-systems/industrial-robots/lbr-iiwa) with

```@example example_1
robot = TORA.create_robot_kuka_iiwa_14(vis)
# hide
```

!!! info
    To load a different robot, check the [Adding a New Robot](@ref Adding-a-new-robot) page of the documentation.

Afterwards, you should be able to see the robot in the viewer:

![Robot spawn](../assets/screenshots/tutorial_02.png)

## Creating a Problem

Having loaded the robot, we are now ready to define the task we want to solve.

TORA.jl uses a Direct Transcription[^1] approach to optimize trajectories.
This technique splits the trajectory into $N$ equally-spaced[^2] segments
```math
    t_I = t_1 < t_2 < \dots < t_M = t_F,
```
where $t_I$ and $t_F$ are the start and final instants, respectively.
This division results in $M = N + 1$ discrete *mesh points* (a.k.a. *knots*), for each of which TORA.jl discretizes the states of the system, as well as the control inputs.

[^1]:
    Betts, John T. [*Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*](https://epubs.siam.org/doi/book/10.1137/1.9780898718577). Society for Industrial and Applied Mathematics, 2010.

[^2]:
    Direct Transcription does not necessarily require equally-spaced segments.

To create a new problem, we use the [`TORA.Problem`](@ref) constructor, which takes three arguments:
1. a [`TORA.Robot`](@ref),
2. the number of *knots* we wish to use for representing the trajectory, and
3. the time step duration between each pair of consecutive knots.

```@setup example_2
```
Suppose we want to optimize a motion with a total duration of *2 seconds*, and that we want to calculate the control inputs to the system at a frequency of *150 Hz*.
```@example example_2
const duration = 2.0  # in seconds
const hz = 150
# hide
```
In that case, the time step duration would be
```@example example_2
dt = 1/150
```
and the total number of *knots* would be given by
```@example example_2
hz * duration + 1
```

Therefore, we create the problem by running
```@example example_1
problem = TORA.Problem(robot, 301, 1/150)
# hide
```

We can use [`TORA.show_problem_info`](@ref) to print a summary of relevant information of a [`TORA.Problem`](@ref).
```@example example_1
TORA.show_problem_info(problem)
```
The summary above shows that the total duration of the motion is 2.0 seconds, just like we wanted.

The summary also shows that there are no constraints defined yet, as we have just now created the problem.

## Defining Constraints

The decision variables of the optimization problem at every *knot* are:
- joint positions,
- joint velocities, and
- joint torques.[^3]

[^3]:
    An exception to this is the last knot, which only discretizes joint positions and joint velocities.

### Bounds of the Decision Variables

The bounds of the decision variables need not be defined.
They are automatically inferred from the URDF model.

### Fixing Values of the Decision Variables

It is possible to fix the values of specific decision variables with:
- [`TORA.fix_joint_positions!`](@ref)
- [`TORA.fix_joint_velocities!`](@ref)
- [`TORA.fix_joint_torques!`](@ref)

Suppose we want to enforce zero joint velocities both at the very *start* of the motion and at the very *end* of the motion. We can specify such constraints with

```@example example_1
# Constrain initial and final joint velocities to zero
TORA.fix_joint_velocities!(problem, robot, 1, zeros(robot.n_v))
TORA.fix_joint_velocities!(problem, robot, problem.num_knots, zeros(robot.n_v))
# hide
```

Let's have a look at the problem summary output again:
```@example example_1
TORA.show_problem_info(problem)
```

The output confirms that there are now *two knots* for which we have fixed specific joint velocities.

### End-effector Constraints

[`TORA.constrain_ee_position!`](@ref) allows us to enforce specific positions for the end-effector of the robot.

If we want the end-effector of the robot to be located at $[0.5, 0.2, 0.3]$ in knot $k = 33$ we would write
```julia
TORA.constrain_ee_position!(problem, 33, [0.5, 0.2, 0.3])
```

But let's do something more interesting for this tutorial... Let's define a problem where the robot must trace a circular path. For that, we just need to call [`TORA.constrain_ee_position!`](@ref) multiple times with the right combination of knot and position such that we describe a circle.

```@example example_1
let CubicTimeScaling(Tf::Number, t::Number) = 3(t / Tf)^2 - 2(t / Tf)^3
    for k = 1:2:problem.num_knots  # For every other knot
        θ = CubicTimeScaling(problem.num_knots - 1, k - 1) * 2π
        pos = [0.5, 0.2 * cos(θ), 0.8 + 0.2 * sin(θ)]
        TORA.constrain_ee_position!(problem, k, pos)
    end
end
```

In the snippet above, `CubicTimeScaling` is a helper function (a cubic polynomial). It allows us to specify a path for the end-effector that accelerates at first, and decelerates near the end. This is a better alternative to tracing the path with constant velocity.

The `for` loop inside the `let` block samples every other knot of the trajectory, and computes the position `pos` of the end-effector at that knot using the angle $\theta$.


Let's print the problem summary once again:
```@example example_1
TORA.show_problem_info(problem)
```

The output correctly shows that we now have end-effector position constraints in 151 knots.

## Providing an Initial Guess

Trajectory Optimization problems can be very challenging to solve. As such, providing a good initial guess (starting point) for the trajectory helps solvers significantly.

For our circle-tracing task, we are going to define a very simple (but reasonably good) initial guess: a static configuration, zero velocities, and zero torques.

First, let's define the static configuration:

```@example example_1
initial_q = [0, 0, 0, -π/2, 0, 0, 0]
# hide
```

We can visualize this configuration by running
```@example example_1
zero!(robot.state)
set_configuration!(robot.state, initial_q)
set_configuration!(robot.mvis, configuration(robot.state))
```

This will update the configuration of the robot in the viewer:

![Robot initial configuration](../assets/screenshots/tutorial_03.png)

We can now define the initial guess for the joint positions with that fixed configuration, repeated for every knot:

```@example example_1
initial_qs = repeat(initial_q, 1, problem.num_knots)
```

For the joint velocities we are going to start with zeroes repeated for every knot:

```@example example_1
initial_vs = zeros(robot.n_v, problem.num_knots)
```

And the same for the joint torques:

```@example example_1
initial_τs = zeros(robot.n_τ, problem.num_knots)
```

We can concatenate these matrices into a single one:
```@example example_1
initial_guess = [initial_qs; initial_vs; initial_τs]
```

Notice that the dimensions are `21×301`, i.e., the dimension of each knot ($\mathbb{R}^{21}$) times the 301 knots.

We can flatten this matrix into a vector with
```@example example_1
initial_guess = vec(initial_guess)
# hide
```

!!! note "Remember"
    Julia follows a column-major convention. See [Access arrays in memory order, along columns](https://docs.julialang.org/en/v1/manual/performance-tips/#man-performance-column-major).

As a very last step, we just need to truncate the last few values that correspond to the control inputs at the last knot. The last knot represents the end of the motion, so we only represent the robot state.

```@example example_1
initial_guess = initial_guess[1:end - robot.n_τ]
# hide
```

## Solving the Problem

```@example example_1
user_options = Dict("print_level" => 0)  # hide
cpu_time, x, solver_log = TORA.solve_with_ipopt(problem, robot, initial_guess=initial_guess, user_options=user_options)  # hide
# hide
```

Once we are happy with the problem definition, we just need to call solve and the optimization will start.

```@example example_1
cpu_time, x, solver_log = TORA.solve_with_ipopt(problem, robot, initial_guess=initial_guess)
# hide
```

!!! info
    There exist additional parameters that you can pass to `TORA.solve_with_*`. See [Advanced Usage](@ref).

## Showing the Results

When the optimization finishes, we can playback the trajectory on the robot shown in the viewer.

```julia
play_trajectory(vis, problem, robot, x)
```

You should be able to see this:

```@raw html
<video src="../assets/videos/tutorial.mp4" autoplay loop muted width="100%" style="margin-bottom: 1em;">Your browser does not support the video tag.</video>
```

We can also plot the positions, velocities, and torques of the obtained trajectory:

```@example example_1
TORA.plot_results(problem, robot, x)
```

Lastly, we can plot the [`TORA.SolverLog`](@ref) (returned by the solve function) to study the evolution of the feasibility error and of the objective function value (if one has been defined) per iteration.

```@example example_1
TORA.plot_log(solver_log)
```

!!! tip "You have reached the end"
    That concludes this tutorial. Congratulations! And thank you for sticking around this far.

```@raw html
<figure>
    <img src="https://thumbs.gfycat.com/DesertedWhiteDotterel-size_restricted.gif" alt="That's all Folks!" width="50%">
</figure>
```
