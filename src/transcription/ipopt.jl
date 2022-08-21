addOption(prob, k, v::String) = Ipopt.AddIpoptStrOption(prob, k, v)
addOption(prob, k, v::Int) = Ipopt.AddIpoptIntOption(prob, k, v)
addOption(prob, k, v::Float64) = Ipopt.AddIpoptNumOption(prob, k, v)

"""
    solve_with_ipopt(problem, robot;
                     initial_guess=Float64[],
                     use_inv_dyn=false,
                     minimise_τ=false,
                     user_options=Dict())

Solve the nonlinear optimization problem with Ipopt.

Further options can be set using the keyword arguments. See [Solver Interfaces](@ref).

# Keyword arguments
- `initial_guess::Vector{Float64}=Float64[]`: the starting point for the solver.
- `use_inv_dyn::Bool=false`: if true, enables the use of inverse dynamics instead of forward dynamics.
- `minimise_τ::Bool=false`: if true, activates a cost function to minimize the joint torques.
- `user_options::Dict=Dict()`: the user options for Ipopt.

See also: [`solve_with_knitro`](@ref)
"""
function solve_with_ipopt(problem::Problem, robot::Robot;
                          initial_guess::Vector{Float64}=Float64[],
                          use_inv_dyn::Bool=false,
                          minimise_τ::Bool=false,
                          user_options::Dict=Dict())
    # # # # # # # # # # # # # # # #
    # Variables and their bounds  #
    # # # # # # # # # # # # # # # #

    n1 = robot.n_q *  problem.num_knots       # positions
    n2 = robot.n_v *  problem.num_knots       # velocities
    n3 = robot.n_τ * (problem.num_knots - 1)  # torques

    n = n1 + n2 + n3  # total number of decision variables

    x_L = Matrix{Float64}(undef, robot.n_q, 0)
    x_U = Matrix{Float64}(undef, robot.n_q, 0)

    for k = 1:problem.num_knots
        if k ∈ keys(problem.fixed_q)
            x_L = [x_L problem.fixed_q[k]]
            x_U = [x_U problem.fixed_q[k]]
        else
            x_L = [x_L robot.q_lo]
            x_U = [x_U robot.q_hi]
        end

        if k ∈ keys(problem.fixed_v)
            x_L = [x_L problem.fixed_v[k]]
            x_U = [x_U problem.fixed_v[k]]
        else
            x_L = [x_L robot.v_lo]
            x_U = [x_U robot.v_hi]
        end

        if k <= problem.num_knots - 1
            if k ∈ keys(problem.fixed_τ)
                x_L = [x_L problem.fixed_τ[k]]
                x_U = [x_U problem.fixed_τ[k]]
            else
                x_L = [x_L robot.τ_lo]
                x_U = [x_U robot.τ_hi]
            end
        end
    end

    # # # # # # # #
    # Constraints #
    # # # # # # # #

    use_m₁ = true  # nonlinear equality: dynamics
    use_m₂ = true  # nonlinear equality: ee xyz-position

    m₁ = !use_m₁ ? 0 : (robot.n_q + robot.n_v) * (problem.num_knots - 1)  # equations of motion
    m₂ = !use_m₂ ? 0 : 3 * length(problem.ee_pos)  # xyz-position (3 NL equality)

    m = m₁ + m₂  # total number of constraints

    ind_con_dyn    = (1:m₁)
    ind_con_ee_pos = (1:m₂) .+ (m₁)

    g_L, g_U = Float64[], Float64[]

    if use_m₁
        append!(g_L, zeros(m₁))
        append!(g_U, zeros(m₁))
    end

    if use_m₂
        knots_con_ee = sort(collect(keys(problem.ee_pos)))
        con_ee = hcat([[problem.ee_pos[k].data...] for k in knots_con_ee]...)
        append!(g_L, vec(con_ee))
        append!(g_U, vec(con_ee))
    end

    @assert length(g_L) == length(g_U)
    @assert eltype(g_L) == eltype(g_U) == Float64

    # dimension of each mesh point
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    if use_inv_dyn
        jacdata_dyn = problem.jacdata_inv_dyn
        dynamics_defects! = inverse_dynamics_defects!
    else
        jacdata_dyn = problem.jacdata_fwd_dyn
        dynamics_defects! = forward_dynamics_defects!
    end

    function eval_g(x, g)
        if use_m₁
            length_c = size(jacdata_dyn.jac, 1)  # length of constraint per pair of knots
            for i = 0:problem.num_knots - 2
                # Calculate the indices of the appropriate decision variables
                ind_vars = range(1 + i * nₓ, length=size(jacdata_dyn.jac, 2))

                # Evaluate constraints
                offset_con = i * length_c
                ind_cons = (1:length_c) .+ offset_con
                @views dynamics_defects!(g[ind_cons], robot, x[ind_vars], problem.dt)
            end
        end

        if use_m₂
            length_c = 3
            for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
                ind_vars = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
                offset_con = (i - 1) * length_c + (m₁)
                ind_cons = (1:length_c) .+ offset_con
                @views ee_position!(g[ind_cons], robot, x[ind_vars])  # constraint evaluation at that point
            end
        end
    end

    jacIndexConsCB = Cint[]
    jacIndexVarsCB = Cint[]

    if use_m₁
        for i = 0:problem.num_knots - 2
            offset_con = i * size(jacdata_dyn.jac, 1)
            ind_cons = rowvals(jacdata_dyn.jac) .+ offset_con
            append!(jacIndexConsCB, ind_cons)

            for j = 1:size(jacdata_dyn.jac, 2)
                ind_vars = fill(j + i * nₓ, length(nzrange(jacdata_dyn.jac, j)))
                append!(jacIndexVarsCB, ind_vars)
            end
        end

        @assert length(jacIndexConsCB) == length(jacIndexVarsCB)
    end

    if use_m₂
        ind_offset = m₁
        for k = knots_con_ee
            inds = rowvals(problem.jacdata_ee_position.jac) .+ ind_offset
            ind_offset += size(problem.jacdata_ee_position.jac, 1)
            append!(jacIndexConsCB, inds)
        end

        for k = knots_con_ee
            vals = vcat([fill(j + (k - 1) * nₓ, length(nzrange(problem.jacdata_ee_position.jac, j)))
                         for j = 1:size(problem.jacdata_ee_position.jac, 2)]...)
            append!(jacIndexVarsCB, vals)
        end

        @assert length(jacIndexConsCB) == length(jacIndexVarsCB)
    end


    function eval_jac_g(x, rows, cols, values::Union{Nothing,Vector{Float64}})
        if isnothing(values)
            for (i, r, c) in zip(1:length(jacIndexConsCB), jacIndexConsCB, jacIndexVarsCB)
                rows[i] = r
                cols[i] = c
            end
        else
            offset_prev = 0

            if use_m₁
                # Dynamics constraints
                for i = 0:problem.num_knots - 2
                    # Calculate the indices of the appropriate decision variables
                    ind_vars = range(1 + i * nₓ, length=size(jacdata_dyn.jac, 2))

                    # Evaluate the Jacobian at that point
                    jacdata_dyn(x[ind_vars])

                    # Pass the Jacobian to Knitro
                    offset_jac = i * jacdata_dyn.length_jac
                    ind_jac = (1:jacdata_dyn.length_jac) .+ offset_jac
                    values[ind_jac] = nonzeros(jacdata_dyn.jac)
                end
                offset_prev += (problem.num_knots - 1) * jacdata_dyn.length_jac
            end

            if use_m₂
                # End-effector constraints
                for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
                    ind_qᵢ = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
                    problem.jacdata_ee_position(x[ind_qᵢ])              # jacobian evaluation at that point
                    offset_jac = (i - 1) * problem.jacdata_ee_position.length_jac + offset_prev
                    ind_jac = (1:problem.jacdata_ee_position.length_jac) .+ offset_jac
                    values[ind_jac] = nonzeros(problem.jacdata_ee_position.jac)  # pass jacobian to Knitro
                end
            end
        end
    end

    # # # # # # #
    # Objective #
    # # # # # # #

    function eval_f(x)
        return 0
    end

    function eval_grad_f(x, grad_f)
        grad_f[:] = zeros(n)
    end

    if minimise_τ
        ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ)
                      for i = (1:problem.num_knots - 1) .- 1]...)
        indexVars, coefs = vec(ind_τ), fill(1 / (problem.num_knots - 1), n3)
        @assert length(indexVars) == length(coefs)

        eval_f = function(x)
            # Minimize τ, i.e., the necessary joint torques.
            return sum(coefs .* x[indexVars] .* x[indexVars])
        end

        eval_grad_f = function(x, grad_f)
            grad_f[:] = zeros(n)
            grad_f[indexVars] .= coefs .* 2x[indexVars]
        end
    end

    # # # # # # # # # #
    # Create Problem  #
    # # # # # # # # # #

    # number of nonzeros in the Jacobian of the constraints
    nele_jac = 0
    nele_jac += !use_m₁ ? 0 : jacdata_dyn.length_jac * (problem.num_knots - 1)
    nele_jac += !use_m₂ ? 0 : problem.jacdata_ee_position.length_jac * length(problem.ee_pos)

    prob = Ipopt.CreateIpoptProblem(n, vec(x_L), vec(x_U), m, g_L, g_U, nele_jac, 0,
                                    eval_f, eval_g, eval_grad_f, eval_jac_g, nothing)

    # # # # # # # # #
    # Initial guess #
    # # # # # # # # #

    if initial_guess |> isempty
        initial_guess = zeros(n)
    end

    @assert length(initial_guess) == n

    # Set starting solution
    prob.x = copy(initial_guess)

    # # # # # # # # #
    # User Options  #
    # # # # # # # # #

    foreach(x -> addOption(prob, x...), user_options)

    solver_log = SolverLog(n)

    function intermediate(alg_mod::Cint, iter_count::Cint, obj_value::Float64, inf_pr::Float64, inf_du::Float64, mu::Float64,
                          d_norm::Float64, regularization_size::Float64, alpha_du::Float64, alpha_pr::Float64, ls_trials::Cint,)
        # Flush the output to the Jupyter notebook cell. Without it,
        # the output will only show up after the solver has terminated.
        flush(stdout)

        update!(solver_log, abs_feas_error=inf_pr, obj_value=obj_value)

        return true
    end

    Ipopt.SetIntermediateCallback(prob, intermediate)

    # # Perform a derivative check.
    # addOption(prob, "derivative_test", "first-order")

    # # # # #
    # Solve #
    # # # # #

    cpu_time = @elapsed status = Ipopt.IpoptSolve(prob)

    # println(Ipopt.ApplicationReturnStatus[status])
    # println(prob.x)
    # println(prob.obj_val)

    return cpu_time, prob.x, solver_log
end

export solve_with_ipopt
