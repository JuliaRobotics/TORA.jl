using PrecompileTools: @setup_workload, @compile_workload

# PrecompileTools allows you to reduce the latency of the first execution of Julia code.
# It is applicable for package developers and for "ordinary users" in their personal workflows.
# See the GitHub repository and docs for more details: https://github.com/JuliaLang/PrecompileTools.jl.

@setup_workload begin

    # Putting some things in `@setup_workload` instead of `@compile_workload` can
    # reduce the size of the precompile file and potentially make loading faster.

    # CAREFUL! Objects defined here will persist in every session including this package.

    @compile_workload begin

        # All calls in this block will be precompiled, regardless of whether they belong to this package or not.

        # Options for the Ipopt numerical solver
        ipopt_options = Dict(
            # "acceptable_compl_inf_tol" => 0.1,
            # "acceptable_constr_viol_tol" => 0.1,
            # "acceptable_dual_inf_tol" => 1.0,
            "acceptable_iter" => 1,
            "acceptable_tol" => 1.0,
            "hessian_approximation" => "limited-memory",
            # "max_cpu_time" => 15.0,
            "max_iter" => 1,
            "mu_strategy" => "monotone",
            "print_level" => 0,  # default: 5
        )

        vis = Visualizer()
        robot = create_robot_franka("panda_arm", vis)
        problem = Problem(robot, 2, 1 / 100)

        initial_q = Float64[0, 0, 0, -π/2, 0, π, 0]

        fix_joint_positions!(problem, robot, 1, initial_q)
        fix_joint_velocities!(problem, robot, 1, zeros(robot.n_v))
        constrain_ee_position!(problem, 1, zeros(3))

        body_name = "panda_link7"  # this is the fixed parent link of "panda_hand_tcp"
        quaternion = QuatRotation(0.27, 0.65, 0.27, 0.65)
        add_constraint_body_orientation!(problem, robot, body_name, 1, quaternion)

        initial_guess = zeros(problem.num_knots * (robot.n_q + robot.n_v + robot.n_τ) - robot.n_τ)

        cpu_time, x, solver_log = solve_with_ipopt(problem, robot, initial_guess=initial_guess, user_options=ipopt_options, use_inv_dyn=false, minimise_τ=false)
        cpu_time, x, solver_log = solve_with_ipopt(problem, robot, initial_guess=initial_guess, user_options=ipopt_options, use_inv_dyn=false, minimise_τ=true)
        cpu_time, x, solver_log = solve_with_ipopt(problem, robot, initial_guess=initial_guess, user_options=ipopt_options, use_inv_dyn=true, minimise_τ=false)

        play_trajectory(vis, problem, robot, x)
        plot_results(problem, robot, x)

        Plots.closeall()
        MeshCat.close_server!(vis.core)

    end

end
