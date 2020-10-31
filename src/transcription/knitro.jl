function solve_with_knitro(problem::Problem, robot::Robot)
    lm = KNITRO.LMcontext()  # Instantiate license manager
    kc = KNITRO.KN_new_lm(lm)  # Create a new Knitro instance

    # # # # # # # # # # # # # # # #
    # Variables and their bounds  #
    # # # # # # # # # # # # # # # #

    n1 = robot.n_q *  problem.num_knots       # positions
    n2 = robot.n_v *  problem.num_knots       # velocities
    n3 = robot.n_τ * (problem.num_knots - 1)  # torques

    n = n1 + n2 + n3  # total number of decision variables
    KNITRO.KN_add_vars(kc, n)  # Add the variables

    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh point

    # Helper ranges
    ind_q = hcat([range(1 + (i * nₓ)                        , length=robot.n_q) for i = (1:problem.num_knots    ) .- 1]...)
    ind_v = hcat([range(1 + (i * nₓ) + robot.n_q            , length=robot.n_v) for i = (1:problem.num_knots    ) .- 1]...)
    ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ) for i = (1:problem.num_knots - 1) .- 1]...)

    KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_q) .- 1), repeat(robot.q_lo, problem.num_knots))  # q lower bounds
    KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_q) .- 1), repeat(robot.q_hi, problem.num_knots))  # q upper bounds

    KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_v) .- 1), repeat(robot.v_lo, problem.num_knots))  # v lower bounds
    KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_v) .- 1), repeat(robot.v_hi, problem.num_knots))  # v upper bounds

    KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_τ) .- 1), repeat(robot.τ_lo, problem.num_knots - 1))  # τ lower bounds
    KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_τ) .- 1), repeat(robot.τ_hi, problem.num_knots - 1))  # τ upper bounds

    # Fixed variables
    for (k, q) ∈ problem.fixed_q KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_q[:,k]) .- 1), q) end
    for (k, v) ∈ problem.fixed_v KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_v[:,k]) .- 1), v) end
    for (k, τ) ∈ problem.fixed_τ KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_τ[:,k]) .- 1), τ) end

    # # # # #
    # Solve #
    # # # # #

    KNITRO.KN_solve(kc)  # Solve the problem

    # Retrieve solution information
    nStatus, objSol, x, duals = KNITRO.KN_get_solution(kc)
    cpu_time = KNITRO.KN_get_solve_time_real(kc)

    KNITRO.KN_free(kc)  # Delete the Knitro solver instance
    KNITRO.KN_release_license(lm)  # Free license manager

    return cpu_time, x
end
