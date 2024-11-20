import PrecompileTools

PrecompileTools.@compile_workload begin
    vis = Visualizer()
    robot = create_robot_franka("panda_arm", vis)
    problem = Problem(robot, 201, 1/100)
    fix_joint_velocities!(problem, robot, 1, zeros(robot.n_v))
    cpu_time, x, solver_log = solve_with_ipopt(problem, robot)
    play_trajectory(vis, problem, robot, x)
    plot_results(problem, robot, x)
    Plots.closeall()
    MeshCat.close_server!(vis.core)
end
