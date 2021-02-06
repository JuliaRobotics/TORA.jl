mutable struct Problem
    num_knots::Int  # Total number of knots
    dt::Float64  # Time step between two knots

    fixed_q::Dict{Int64,Array{Float64}}  # Fixed joint positions
    fixed_v::Dict{Int64,Array{Float64}}  # Fixed joint velocities
    fixed_τ::Dict{Int64,Array{Float64}}  # Fixed joint torques

    ee_pos::Dict{Int64,Point{3,Float64}}  # End-effector target positions

    # Jacobian data (e.g., for sparse Jacobian of dynamics with AD)
    jacdata_fwd_dyn::JacobianData
    jacdata_inv_dyn::JacobianData
    jacdata_ee_position::JacobianData

    function Problem(robot, num_knots, dt)
        @assert num_knots > 0
        @assert dt > 0

        input, output = rand(2 * (robot.n_q + robot.n_v) + robot.n_τ), rand(robot.n_q + robot.n_v)
        jacdata_fwd_dyn = JacobianData((out, x) -> forward_dynamics_defects!(out, robot, x, dt), output, input)

        input, output = rand(2 * (robot.n_q + robot.n_v) + robot.n_τ), rand(robot.n_q + robot.n_τ)
        jacdata_inv_dyn = JacobianData((out, x) -> inverse_dynamics_defects!(out, robot, x, dt), output, input)

        input, output = rand(robot.n_q), rand(3)
        jacdata_ee_position = JacobianData((out, x) -> ee_position!(out, robot, x), output, input)

        new(
            num_knots, dt,
            Dict{Int64,Array{Float64}}(),
            Dict{Int64,Array{Float64}}(),
            Dict{Int64,Array{Float64}}(),
            Dict{Int64,Point{3,Float64}}(),
            jacdata_fwd_dyn,
            jacdata_inv_dyn,
            jacdata_ee_position
        )
    end
end

"""
    fix_joint_positions!(problem, robot, knot, q)

Returns a.
"""
function fix_joint_positions!(problem::Problem, robot::Robot, knot, q)
    @assert 1 <= knot <= problem.num_knots
    @assert all(robot.q_lo .<= q .<= robot.q_hi)
    problem.fixed_q[knot] = q
    return problem
end

"""
    fix_joint_velocities!(problem, robot, knot, v)

To do.
"""
function fix_joint_velocities!(problem::Problem, robot::Robot, knot, v)
    @assert 1 <= knot <= problem.num_knots
    @assert all(robot.v_lo .<= v .<= robot.v_hi)
    problem.fixed_v[knot] = v
    return problem
end

"""
    fix_joint_torques!(problem, robot, knot, τ)

To do.
"""
function fix_joint_torques!(problem::Problem, robot::Robot, knot, τ)
    @assert 1 <= knot <= problem.num_knots - 1
    @assert all(robot.τ_lo .<= τ .<= robot.τ_hi)
    problem.fixed_τ[knot] = τ
    return problem
end

"""
    constrain_ee_position!(problem, knot, position)

To do.
"""
function constrain_ee_position!(problem::Problem, knot, position)
    @assert 1 <= knot <= problem.num_knots
    @assert length(position) == 3
    problem.ee_pos[knot] = position
    return problem
end

function show_problem_info(problem::Problem)
    t = (problem.num_knots - 1) * problem.dt

    println("Motion duration ...................................... $(t) seconds")
    println("Number of knots with constrained joint positions ..... $(length(problem.fixed_q))")
    println("Number of knots with constrained joint velocities .... $(length(problem.fixed_v))")
    println("Number of knots with constrained joint torques ....... $(length(problem.fixed_τ))")
    println("Number of knots with constrained ee position ......... $(length(problem.ee_pos))")
end

export
    Problem,
    fix_joint_positions!,
    fix_joint_velocities!,
    fix_joint_torques!,
    constrain_ee_position!,
    show_problem_info
