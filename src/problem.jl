"""
A `Problem` stores the high-level description of a motion planning problem.

This is the main structure of the package.
The idea is for users to start with an empty problem for a given robot.
Then, in order to specify the requirements for the motion they want to compute, they can add different types of constraints, one by one.
See [Creating a Problem](@ref) and [Defining Constraints](@ref) for detailed examples.

Eventually, the problem becomes a complex description with all the requirements.
Once the description is finalized, it can be optimized.
This happens in one of the `TORA.solve_with_*` functions, which does the heavy-lifting of converting a `Problem` (the high-level description) into an [NLP](https://en.wikipedia.org/wiki/Nonlinear_programming) problem that can be solved using off-the-shelf solvers, such as [Ipopt](https://github.com/coin-or/Ipopt) and [Knitro](https://www.artelys.com/solvers/knitro/).

See also: [`solve_with_ipopt`](@ref), [`solve_with_knitro`](@ref)
"""
mutable struct Problem
    num_knots::Int  # Total number of knots
    dt::Float64  # Time step between two knots

    fixed_q::Dict{Int64,Vector{Float64}}  # Fixed joint positions
    fixed_v::Dict{Int64,Vector{Float64}}  # Fixed joint velocities
    fixed_τ::Dict{Int64,Vector{Float64}}  # Fixed joint torques

    ee_pos::Dict{Int64,Point{3,Float64}}  # End-effector target positions

    # Jacobian data (e.g., for sparse Jacobian of dynamics with AD)
    jacdata_fwd_dyn::JacobianData
    jacdata_inv_dyn::JacobianData
    jacdata_ee_position::JacobianData

    @doc """
        Problem(robot, num_knots, dt)

    Create an empty `Problem`.

    # Arguments
    - `robot::Robot`: the robot associated with this `Problem`.
    - `num_knots::Int`: the number of knots used to represent the motion.
    - `dt::Float64`: the time step duration between each two consecutive knots.

    See also: [`show_problem_info`](@ref)
    """
    function Problem(robot::Robot, num_knots::Int, dt::Float64)
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
            Dict{Int64,Vector{Float64}}(),
            Dict{Int64,Vector{Float64}}(),
            Dict{Int64,Vector{Float64}}(),
            Dict{Int64,Point{3,Float64}}(),
            jacdata_fwd_dyn,
            jacdata_inv_dyn,
            jacdata_ee_position
        )
    end
end

"""
    fix_joint_positions!(problem, robot, knot, q)

Fix the joint positions of the `robot` at a specific `knot`.

See also: [`fix_joint_velocities!`](@ref), [`fix_joint_torques!`](@ref)
"""
function fix_joint_positions!(problem::Problem, robot::Robot, knot, q)
    @assert 1 <= knot <= problem.num_knots
    @assert all(robot.q_lo .<= q .<= robot.q_hi)
    problem.fixed_q[knot] = q
    return problem
end

"""
    fix_joint_velocities!(problem, robot, knot, v)

Fix the joint velocities of the `robot` at a specific `knot`.

See also: [`fix_joint_positions!`](@ref), [`fix_joint_torques!`](@ref)
"""
function fix_joint_velocities!(problem::Problem, robot::Robot, knot, v)
    @assert 1 <= knot <= problem.num_knots
    @assert all(robot.v_lo .<= v .<= robot.v_hi)
    problem.fixed_v[knot] = v
    return problem
end

"""
    fix_joint_torques!(problem, robot, knot, τ)

Fix the joint torques of the `robot` at a specific `knot`.

See also: [`fix_joint_positions!`](@ref), [`fix_joint_velocities!`](@ref)
"""
function fix_joint_torques!(problem::Problem, robot::Robot, knot, τ)
    @assert 1 <= knot <= problem.num_knots - 1
    @assert all(robot.τ_lo .<= τ .<= robot.τ_hi)
    problem.fixed_τ[knot] = τ
    return problem
end

"""
    constrain_ee_position!(problem, knot, position)

Fix the end-efector positions of the `robot` at a specific `knot`.

See also: [`fix_joint_positions!`](@ref), [`fix_joint_velocities!`](@ref), [`fix_joint_torques!`](@ref)
"""
function constrain_ee_position!(problem::Problem, knot, position)
    @assert 1 <= knot <= problem.num_knots
    @assert length(position) == 3
    problem.ee_pos[knot] = position
    return problem
end

"""
    show_problem_info(problem)

Output a summary of the problem, including the number of knots with constraints.
"""
function show_problem_info(problem::Problem)
    t = (problem.num_knots - 1) * problem.dt

    println("Motion duration ...................................... $(t) seconds")
    println("Number of knots ...................................... $(problem.num_knots)")
    println("Number of knots with constrained joint positions ..... $(length(problem.fixed_q))")
    println("Number of knots with constrained joint velocities .... $(length(problem.fixed_v))")
    println("Number of knots with constrained joint torques ....... $(length(problem.fixed_τ))")
    println("Number of knots with constrained ee position ......... $(length(problem.ee_pos))")
end

"""
    export_trajectory(file, problem, robot, x)

Export a trajectory to a `.npz` file.
"""
function export_trajectory(file::String, problem::Problem, robot::Robot, x)
    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh point

    ts = range(0, length=problem.num_knots, step=problem.dt)

    # Helper ranges
    ind_q = hcat([range(1 + (i * nₓ)                        , length=robot.n_q) for i = (1:problem.num_knots    ) .- 1]...)
    ind_v = hcat([range(1 + (i * nₓ) + robot.n_q            , length=robot.n_v) for i = (1:problem.num_knots    ) .- 1]...)
    ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ) for i = (1:problem.num_knots - 1) .- 1]...)

    npzwrite(file, Dict(
        "ts" => ts,
        "qs" => x[ind_q],
        "vs" => x[ind_v],
        "τs" => x[ind_τ],
    ))
end

export
    Problem,
    fix_joint_positions!,
    fix_joint_velocities!,
    fix_joint_torques!,
    constrain_ee_position!,
    show_problem_info,
    export_trajectory
