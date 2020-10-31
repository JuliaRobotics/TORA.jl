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

    m = 0
    g_L = Float64[]
    g_U = Float64[]

    function eval_g(x, g)
        g[:] .= 0
    end

    function eval_jac_g(x, mode, rows, cols, values)
        if mode == :Structure
        else
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

    prob = Ipopt.createProblem(n, vec(x_L), vec(x_U), m, g_L, g_U, 0, 0,
                               eval_f, eval_g, eval_grad_f, eval_jac_g)

    prob.x = zeros(n)  # Set starting solution

    # # # # # # # # #
    # User Options  #
    # # # # # # # # #

    addOption(prob, "hessian_approximation", "limited-memory")

    # # # # #
    # Solve #
    # # # # #

    status = Ipopt.solveProblem(prob)

    # println(Ipopt.ApplicationReturnStatus[status])
    # println(prob.x)
    # println(prob.obj_val)

    return prob.x
end
