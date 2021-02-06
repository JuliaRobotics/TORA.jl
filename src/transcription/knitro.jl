using .KNITRO

function solve_with_knitro(problem::Problem, robot::Robot;
                           initial_guess::Array{Float64}=Float64[],
                           use_inv_dyn::Bool=false,
                           minimise_τ::Bool=false,
                           user_options::Dict=Dict())
    lm = KNITRO.LMcontext()  # Instantiate license manager
    kc = KNITRO.KN_new_lm(lm)  # Create a new Knitro instance

    # # # # # # # # # # # # # # # #
    # Variables and their bounds  #
    # # # # # # # # # # # # # # # #

    n1 = robot.n_q *  problem.num_knots       # positions
    n2 = robot.n_v *  problem.num_knots       # velocities
    n3 = robot.n_τ * (problem.num_knots - 1)  # torques

    n = n1 + n2 + n3  # total number of decision variables
    KNITRO.KN_add_vars(kc, n)  # add the variables

    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh point

    # Helper ranges
    ind_q = hcat([range(1 + (i * nₓ)                        , length=robot.n_q) for i = (1:problem.num_knots    ) .- 1]...)
    ind_v = hcat([range(1 + (i * nₓ) + robot.n_q            , length=robot.n_v) for i = (1:problem.num_knots    ) .- 1]...)
    ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ) for i = (1:problem.num_knots - 1) .- 1]...)

    KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_q) .- 1), repeat(robot.q_lo, problem.num_knots))  # q lower bounds
    KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_q) .- 1), repeat(robot.q_hi, problem.num_knots))  # q upper bounds

    KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_v) .- 1), repeat(robot.v_lo, problem.num_knots))  # v lower bounds
    KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_v) .- 1), repeat(robot.v_hi, problem.num_knots))  # v upper bounds

    if !isempty(ind_τ)
        KNITRO.KN_set_var_lobnds(kc, Cint.(vec(ind_τ) .- 1), repeat(robot.τ_lo, problem.num_knots - 1))  # τ lower bounds
        KNITRO.KN_set_var_upbnds(kc, Cint.(vec(ind_τ) .- 1), repeat(robot.τ_hi, problem.num_knots - 1))  # τ upper bounds
    end

    # Fixed variables
    for (k, q) ∈ problem.fixed_q KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_q[:,k]) .- 1), q) end
    for (k, v) ∈ problem.fixed_v KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_v[:,k]) .- 1), v) end
    for (k, τ) ∈ problem.fixed_τ KNITRO.KN_set_var_fxbnds(kc, Cint.(vec(ind_τ[:,k]) .- 1), τ) end

    # # # # # # # #
    # Constraints #
    # # # # # # # #

    use_m₁ = true  # nonlinear equality: dynamics
    use_m₂ = true  # nonlinear equality: ee xyz-position

    m₁ = !use_m₁ ? 0 : (robot.n_q + robot.n_v) * (problem.num_knots - 1)  # equations of motion
    m₂ = !use_m₂ ? 0 : 3 * length(problem.ee_pos)  # xyz-position (3 NL equality)

    m = m₁ + m₂  # total number of constraints
    KNITRO.KN_add_cons(kc, m)  # add the constraints

    ind_con_dyn    = (1:m₁)
    ind_con_ee_pos = (1:m₂) .+ (m₁)

    if use_m₁
        KNITRO.KN_set_con_eqbnds(kc, collect(Cint, ind_con_dyn .- 1), zeros(m₁))

        if use_inv_dyn
            jac = problem.jacdata_inv_dyn.jac
            cb_eval_fc_con = make_cb_eval_fc_con_dyn(problem, robot, problem.jacdata_inv_dyn, inverse_dynamics_defects!)
            cb_eval_ga_con = make_cb_eval_ga_con_dyn(problem, robot, problem.jacdata_inv_dyn)
        else
            jac = problem.jacdata_fwd_dyn.jac
            cb_eval_fc_con = make_cb_eval_fc_con_dyn(problem, robot, problem.jacdata_fwd_dyn, forward_dynamics_defects!)
            cb_eval_ga_con = make_cb_eval_ga_con_dyn(problem, robot, problem.jacdata_fwd_dyn)
        end

        cb = KNITRO.KN_add_eval_callback(kc, false, collect(Cint, ind_con_dyn .- 1), cb_eval_fc_con)

        jacIndexConsCB = Cint.(hcat([rowvals(jac) .+ i * size(jac, 1)
                                     for i = 0:problem.num_knots - 2]...) .- 1)

        jacIndexVarsCB = Cint.(hcat([vcat([fill(j + i * nₓ, length(nzrange(jac, j)))
                                           for j = 1:size(jac, 2)]...)
                                     for i = 0:problem.num_knots - 2]...) .- 1)

        @assert length(jacIndexConsCB) == length(jacIndexVarsCB)
        @assert eltype(jacIndexConsCB) == eltype(jacIndexVarsCB) == Cint

        KNITRO.KN_set_cb_grad(kc, cb, cb_eval_ga_con,
                              nV=0, objGradIndexVars=C_NULL,
                              jacIndexCons=jacIndexConsCB,
                              jacIndexVars=jacIndexVarsCB)
    end

    if use_m₂
        knots_con_ee = sort(collect(keys(problem.ee_pos)))

        con_ee = Array{Float64,2}(undef, 3, 0)

        for k in knots_con_ee
            con_ee = [con_ee problem.ee_pos[k]]
        end

        @assert length(ind_con_ee_pos) == length(con_ee)

        KNITRO.KN_set_con_eqbnds(kc, collect(Cint, ind_con_ee_pos .- 1), vec(con_ee))

        cb = KNITRO.KN_add_eval_callback(kc, false, collect(Cint, ind_con_ee_pos .- 1), cb_eval_fc_con_ee)

        ind_offset = 0
        jacIndexConsCB = Cint[]
        for k = knots_con_ee
            inds = rowvals(problem.jacdata_ee_position.jac) .+ ind_offset
            ind_offset += size(problem.jacdata_ee_position.jac, 1)
            append!(jacIndexConsCB, inds)
        end
        jacIndexConsCB = Cint.(jacIndexConsCB .+ (m₁) .- 1)

        jacIndexVarsCB = Cint[]
        for k = knots_con_ee
            vals = vcat([fill(j + (k - 1) * nₓ, length(nzrange(problem.jacdata_ee_position.jac, j)))
                         for j = 1:size(problem.jacdata_ee_position.jac, 2)]...)
            append!(jacIndexVarsCB, vals)
        end
        jacIndexVarsCB = Cint.(jacIndexVarsCB .- 1)

        @assert length(jacIndexConsCB) == length(jacIndexVarsCB)
        @assert eltype(jacIndexConsCB) == eltype(jacIndexVarsCB) == Cint

        KNITRO.KN_set_cb_grad(kc, cb, cb_eval_ga_con_ee,
                              nV=0, objGradIndexVars=C_NULL,
                              jacIndexCons=jacIndexConsCB,
                              jacIndexVars=jacIndexVarsCB)

        KNITRO.KN_set_cb_user_params(kc, cb, (problem, robot))
    end

    # # # # # # #
    # Objective #
    # # # # # # #

    if minimise_τ
        # Minimise τ, i.e., the necessary joint torques.
        indexVars, coefs = Cint.(vec(ind_τ) .- 1), fill(1 / (problem.num_knots - 1), n3)
        KNITRO.KN_add_obj_quadratic_struct(kc, indexVars, indexVars, coefs)

        KNITRO.KN_set_obj_goal(kc, KNITRO.KN_OBJGOAL_MINIMIZE)
    end

    # # # # # # # # #
    # Initial guess #
    # # # # # # # # #

    if !isempty(initial_guess)
        KNITRO.KN_set_var_primal_init_values(kc, vec(initial_guess))
    else
        KNITRO.KN_set_var_primal_init_values(kc, zeros(n))
    end

    # # # # # # # # #
    # User Options  #
    # # # # # # # # #

    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_HESSOPT, KNITRO.KN_HESSOPT_LBFGS)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_ALGORITHM, KNITRO.KN_ALG_BAR_DIRECT)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_BAR_MURULE, KNITRO.KN_BAR_MURULE_ADAPTIVE)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_LINSOLVER, KNITRO.KN_LINSOLVER_MA57)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_FEASTOL, 1.0e-3)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_FTOL, 1.0e-6)
    KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_MAXTIMECPU, 10.0)

    solver_log = SolverLog(n)

    function callbackNewPoint(kc, x, duals, userParams)
        print("")  # Flush the output to the Jupyter notebook cell

        abs_feas_error = KNITRO.KN_get_abs_feas_error(kc)
        abs_opt_error = KNITRO.KN_get_abs_opt_error(kc)
        obj_value = KNITRO.KN_get_obj_value(kc)

        fc_evals = KNITRO.KN_get_number_FC_evals(kc)
        ga_evals = KNITRO.KN_get_number_GA_evals(kc)

        update!(solver_log, x, abs_feas_error, abs_opt_error, obj_value, fc_evals, ga_evals)

        return 0
    end
    
    KNITRO.KN_set_newpt_callback(kc, callbackNewPoint)

    # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # # Perform a derivative check.
    # KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_DERIVCHECK, KNITRO.KN_DERIVCHECK_FIRST)
    # KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_DERIVCHECK_TERMINATE, KNITRO.KN_DERIVCHECK_STOPALWAYS)
    # # KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_DERIVCHECK_TOL, 1.0e-4)  # Default value: 1.0e-6
    # KNITRO.KN_set_param(kc, KNITRO.KN_PARAM_DERIVCHECK_TYPE, KNITRO.KN_DERIVCHECK_CENTRAL)

    # # # # #
    # Solve #
    # # # # #

    KNITRO.KN_solve(kc)  # Solve the problem

    # Retrieve solution information
    nStatus, objSol, x, duals = KNITRO.KN_get_solution(kc)
    cpu_time = KNITRO.KN_get_solve_time_real(kc)

    KNITRO.KN_free(kc)  # Delete the Knitro solver instance
    KNITRO.KN_release_license(lm)  # Free license manager

    return cpu_time, x, solver_log
end

export solve_with_knitro
