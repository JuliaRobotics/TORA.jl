#!/usr/bin/env python3

import numpy as np
import time

from juliacall import Main as jl, convert as jlconvert


def init():
    """
    Initialize the Julia environment, load the TORA.jl package, and create a robot model.
    """
    # Load package dependencies
    jl.seval("using MeshCat")
    jl.seval("using Rotations")
    jl.seval("using TORA")

    # Call the greet() function from the TORA.jl package
    jl.TORA.greet()

    jl_vis = jl.MeshCat.Visualizer()
    # jl.open(jl_vis)  # NOT WORKING

    jl_robot = jl.TORA.create_robot_franka("panda_arm", jl_vis)

    return jl_robot


def prepare_initial_guess(jl_problem, jl_robot, py_q):
    # Start by assuming zero positions, velocities, and torques for all the knots of the trajectory
    py_initial_qs = np.zeros((jl_problem.num_knots, jl_robot.n_q))
    py_initial_vs = np.zeros((jl_problem.num_knots, jl_robot.n_v))
    py_initial_τs = np.zeros((jl_problem.num_knots, jl_robot.n_τ))

    # To improve the initial guess for the robot joint positions, we can take
    # the current robot configuration and simply repeat it for all the knots
    py_initial_qs = np.tile(py_q, (jl_problem.num_knots, 1))

    # Concatenate the initial guess
    py_initial_guess = np.concatenate((py_initial_qs, py_initial_vs, py_initial_τs), axis=1)

    # Flatten the numpy array
    py_initial_guess = py_initial_guess.flatten()

    # Truncate the torques at the last knot of the trajectory from the initial guess
    py_initial_guess = py_initial_guess[: -jl_robot.n_τ]

    # Convert the numpy array to a Julia array
    jl_initial_guess = jlconvert(jl.Vector[jl.Float64], py_initial_guess)

    return jl_initial_guess


def main():
    jl.println("Hello from Julia!")

    jl_robot = init()

    trajectory_discretisation = 10  # in Hertz
    trajectory_duration = 2.0  # in seconds

    trajectory_num_knots = int(trajectory_discretisation * trajectory_duration) + 1  # in units (number of knots)
    trajectory_dt = 1 / trajectory_discretisation  # in seconds

    # Options for the Ipopt numerical solver
    ipopt_options = jl.Dict()
    ipopt_options["hessian_approximation"] = "limited-memory"  # L-BFGS
    ipopt_options["max_cpu_time"] = 2.0  # in seconds
    ipopt_options["mu_strategy"] = "adaptive"  # monotone, adaptive, quality-function
    # ipopt_options["acceptable_compl_inf_tol"] = 0.1
    # ipopt_options["acceptable_constr_viol_tol"] = 0.1
    # ipopt_options["acceptable_dual_inf_tol"] = 1.0
    ipopt_options["acceptable_iter"] = 1
    ipopt_options["acceptable_tol"] = 1.0
    # ipopt_options["print_level"] = 3  # default: 5

    # Starting configuration of the robot
    py_q_start = [0, 0, 0, -np.pi / 2, 0, np.pi, 0]

    done = False

    while not done:
        # Create a problem instance
        jl_problem = jl.TORA.Problem(jl_robot, trajectory_num_knots, trajectory_dt)

        # Get the current robot state
        # (but for this example, let's use a nominal configuration and random velocities)
        py_q = py_q_start
        py_v = np.random.rand(jl_robot.n_v)

        # Constrain the initial joint positions and velocities to the current state of the robot
        jl.TORA.fix_joint_positions_b(jl_problem, jl_robot, 1, py_q)
        jl.TORA.fix_joint_velocities_b(jl_problem, jl_robot, 1, py_v)

        # Constrain the final joint velocities to zero
        jl.TORA.fix_joint_velocities_b(jl_problem, jl_robot, jl_problem.num_knots, np.zeros(jl_robot.n_v))

        # Define the end-effector position that we want to achieve
        py_eff_pos = [0.5, 0.2, 0.8]

        # Constrain the end-effector position at the last knot of the trajectory
        jl.TORA.constrain_ee_position_b(jl_problem, jl_problem.num_knots, py_eff_pos)

        # Define the end-effector orientation that we want to achieve
        body_name = "panda_link7"  # this is the fixed parent link of "panda_hand_tcp"
        jl_quaternion = jl.QuatRotation(0.27, 0.65, 0.27, 0.65)  # Quaternion order is (w, x, y, z)

        # Constrain the end-effector orientation at the last knot of the trajectory
        jl.TORA.add_constraint_body_orientation_b(jl_problem, jl_robot, body_name, jl_problem.num_knots, jl_quaternion)

        # Show a summary of the problem instance (this can be commented out)
        jl.TORA.show_problem_info(jl_problem)

        # Prepare the initial guess for the solver
        jl_initial_guess = prepare_initial_guess(jl_problem, jl_robot, py_q)

        jl_cpu_time, jl_x, jl_solver_log = jl.TORA.solve_with_ipopt(
            jl_problem,
            jl_robot,
            initial_guess=jl_initial_guess,
            user_options=ipopt_options,
            use_inv_dyn=True,
            minimise_τ=False,
        )

        # Unpack the solution `x` into joint positions, velocities, and torques
        jl_qs, jl_vs, jl_τs = jl.TORA.unpack_x(jl_x, jl_robot, jl_problem)

        # Convert the Julia arrays to numpy arrays
        py_qs = np.array(jl_qs)
        py_vs = np.array(jl_vs)
        py_τs = np.array(jl_τs)

        # Do something with the result (e.g., control the real robot)
        # (...)

        # Sleep for 1 second before the next iteration
        time.sleep(1)


if __name__ == "__main__":
    main()
