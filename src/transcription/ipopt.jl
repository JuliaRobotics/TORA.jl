function solve_with_ipopt(problem::Problem, robot::Robot)
    # # # # # # # # # # # # # # # #
    # Variables and their bounds  #
    # # # # # # # # # # # # # # # #

    n1 = robot.n_q *  problem.num_knots       # positions
    n2 = robot.n_v *  problem.num_knots       # velocities
    n3 = robot.n_τ * (problem.num_knots - 1)  # torques

    n = n1 + n2 + n3  # total number of decision variables

    x_L = Array{Float64,2}(undef, robot.n_q, 0)
    x_U = Array{Float64,2}(undef, robot.n_q, 0)

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

    use_m₁ = false  # nonlinear equality: dynamics
    use_m₂ = true  # nonlinear equality: ee xyz-position

    m₁ = !use_m₁ ? 0 : (robot.n_q + robot.n_v) * (problem.num_knots - 1)  # equations of motion
    m₂ = !use_m₂ ? 0 : 3 * length(problem.ee_pos)  # xyz-position (3 NL equality)

    m = m₁ + m₂  # total number of constraints

    ind_con_dyn    = (1:m₁)
    ind_con_ee_pos = (1:m₂) .+ (m₁)

    g_L, g_U = Float64[], Float64[]

    if use_m₂
        knots_con_ee = sort(collect(keys(problem.ee_pos)))
        con_ee = hcat([[problem.ee_pos[k].data...] for k in knots_con_ee]...)
        append!(g_L, vec(con_ee))
        append!(g_U, vec(con_ee))
    end

    # dimension of each mesh point
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    function eval_g(x, g)
        length_c = 3

        for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
            ind_qᵢ = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
            ind_c = (1:length_c) .+ ((i - 1) * length_c)
            @views ee_position!(g[ind_c], robot, x[ind_qᵢ])  # constraint evaluation at that point
        end
    end

    ind_offset = 0
    jacIndexConsCB = Cint[]
    for k = knots_con_ee
        inds = rowvals(problem.jacdata_ee_position.jac) .+ ind_offset
        ind_offset += size(problem.jacdata_ee_position.jac, 1)
        append!(jacIndexConsCB, inds)
    end
    jacIndexConsCB = Cint.(jacIndexConsCB .+ (m₁))

    jacIndexVarsCB = Cint[]
    for k = knots_con_ee
        vals = vcat([fill(j + (k - 1) * nₓ, length(nzrange(problem.jacdata_ee_position.jac, j)))
                     for j = 1:size(problem.jacdata_ee_position.jac, 2)]...)
        append!(jacIndexVarsCB, vals)
    end
    jacIndexVarsCB = Cint.(jacIndexVarsCB)

    @assert length(jacIndexConsCB) == length(jacIndexVarsCB)

    function eval_jac_g(x, mode, rows, cols, values)
        if mode == :Structure
            for (i, r, c) in zip(1:length(jacIndexConsCB), jacIndexConsCB, jacIndexVarsCB)
                rows[i] = r
                cols[i] = c
            end
        else
            for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
                ind_qᵢ = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
                problem.jacdata_ee_position(x[ind_qᵢ])              # jacobian evaluation at that point
                ind_jac = (1:problem.jacdata_ee_position.length_jac) .+ ((i - 1) * problem.jacdata_ee_position.length_jac)
                values[ind_jac] = nonzeros(problem.jacdata_ee_position.jac)  # pass jacobian to Knitro
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
        grad_f[:] .= 0
    end

    # # # # # # # # # #
    # Create Problem  #
    # # # # # # # # # #

    nele_jac = problem.jacdata_ee_position.length_jac * length(problem.ee_pos)

    prob = Ipopt.createProblem(n, vec(x_L), vec(x_U), m, g_L, g_U, nele_jac, 0,
                               eval_f, eval_g, eval_grad_f, eval_jac_g)

    prob.x = zeros(n)  # Set starting solution

    # # # # # # # # #
    # User Options  #
    # # # # # # # # #

    addOption(prob, "hessian_approximation", "limited-memory")
    addOption(prob, "mu_strategy", "adaptive")
    addOption(prob, "linear_solver", "ma57")
    addOption(prob, "max_cpu_time", 10.0)

    # # # # #
    # Solve #
    # # # # #

    status = Ipopt.solveProblem(prob)
    cpu_time = 0

    # println(Ipopt.ApplicationReturnStatus[status])
    # println(prob.x)
    # println(prob.obj_val)

    return cpu_time, prob.x
end
