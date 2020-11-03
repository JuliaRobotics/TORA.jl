function explicit_euler_step(qᵢ, vᵢ, v̇ᵢ, h)
    qᵢ₊₁ = qᵢ + h * vᵢ
    vᵢ₊₁ = vᵢ + h * v̇ᵢ

    [qᵢ₊₁
     vᵢ₊₁]
end

function semi_implicit_euler_step(qᵢ, vᵢ, v̇ᵢ, h)
    vᵢ₊₁ = vᵢ + h * v̇ᵢ
    qᵢ₊₁ = qᵢ + h * vᵢ₊₁

    [qᵢ₊₁
     vᵢ₊₁]
end

function forward_dynamics_defects!(defects, robot, x::AbstractArray{T}, h) where {T}
    # Indices of the decision variables
    ind_qᵢ   = (1:robot.n_q)
    ind_vᵢ   = (1:robot.n_v) .+ (robot.n_q)
    ind_τᵢ   = (1:robot.n_τ) .+ (robot.n_q + robot.n_v)
    ind_qᵢ₊₁ = (1:robot.n_q) .+ (robot.n_q + robot.n_v + robot.n_τ)
    ind_vᵢ₊₁ = (1:robot.n_v) .+ (robot.n_q + robot.n_v + robot.n_τ + robot.n_q)

    # Retrieve values of the decision variables
    qᵢ  , vᵢ  , τᵢ = x[ind_qᵢ  ], x[ind_vᵢ  ], x[ind_τᵢ]
    qᵢ₊₁, vᵢ₊₁     = x[ind_qᵢ₊₁], x[ind_vᵢ₊₁]

    # Update robot state
    state = robot.statecache[T]
    set_configuration!(state, qᵢ)
    set_velocity!(state, vᵢ)

    # Compute forward dynamics
    result = robot.dynamicsresultcache[T]
    dynamics!(result, state, τᵢ)
    v̇ᵢ = result.v̇

    # Integrate the robot state
    xᵢ₊₁_integrated = semi_implicit_euler_step(qᵢ, vᵢ, v̇ᵢ, h)

    # Next robot state discretised
    xᵢ₊₁_discretised = [qᵢ₊₁
                        vᵢ₊₁]

    # Evaluate state defects (joint positions and joint velocities)
    copyto!(defects, xᵢ₊₁_integrated - xᵢ₊₁_discretised)
end

function inverse_dynamics_defects!(defects, robot, x::AbstractArray{T}, h) where {T}
    # Indices of the decision variables
    ind_qᵢ   = (1:robot.n_q)
    ind_vᵢ   = (1:robot.n_v) .+ (robot.n_q)
    ind_τᵢ   = (1:robot.n_τ) .+ (robot.n_q + robot.n_v)
    ind_qᵢ₊₁ = (1:robot.n_q) .+ (robot.n_q + robot.n_v + robot.n_τ)
    ind_vᵢ₊₁ = (1:robot.n_v) .+ (robot.n_q + robot.n_v + robot.n_τ + robot.n_q)

    # Retrieve values of the decision variables
    qᵢ  , vᵢ  , τᵢ = x[ind_qᵢ  ], x[ind_vᵢ  ], x[ind_τᵢ]
    qᵢ₊₁, vᵢ₊₁     = x[ind_qᵢ₊₁], x[ind_vᵢ₊₁]

    # Update robot state
    state = robot.statecache[T]
    set_configuration!(state, qᵢ)
    set_velocity!(state, vᵢ)

    # Implicit joint accelerations
    v̇ᵢ = similar(velocity(state))
    copyto!(v̇ᵢ, (vᵢ₊₁ - vᵢ) / h)

    # Compute inverse dynamics
    torques = similar(velocity(state))  # create a SegmentedVector instance
    dynamics_result = robot.dynamicsresultcache[T]
    wrenches = dynamics_result.jointwrenches
    accelerations = dynamics_result.accelerations
    inverse_dynamics!(torques, wrenches, accelerations, state, v̇ᵢ)

    # position_defects = (qᵢ + h * vᵢ) - qᵢ₊₁  # Explicit Euler
    position_defects = (qᵢ + h * vᵢ₊₁) - qᵢ₊₁  # Semi-Implicit Euler
    torque_defects = torques - τᵢ

    # Evaluate defects (joint positions and joint torques)
    copyto!(defects, [position_defects
                        torque_defects])
end

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

function cb_eval_fc_con_fwd_dyn(kc, cb, evalRequest, evalResult, userParams)
    x = evalRequest.x
    problem, robot = userParams

    # dimension of each mesh points
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    length_c = robot.n_q + robot.n_v

    for i = 0:problem.num_knots - 2
        ind_cons = (1:length_c) .+ i * length_c
        ind_vars = range(1 + i * nₓ, length=2 * (robot.n_q + robot.n_v) + robot.n_τ)
        @views forward_dynamics_defects!(evalResult.c[ind_cons], robot, x[ind_vars], problem.dt)
    end

    return 0
end

function cb_eval_ga_con_fwd_dyn(kc, cb, evalRequest, evalResult, userParams)
    x = evalRequest.x
    problem, robot = userParams

    # dimension of each mesh points
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    for i = 0:problem.num_knots - 2
        ind_vars = range(1 + i * nₓ, length=2 * (robot.n_q + robot.n_v) + robot.n_τ)

        problem.jacdata_fwd_dyn(x[ind_vars])

        offset_jac = i * problem.jacdata_fwd_dyn.length_jac
        ind_jac = (1:problem.jacdata_fwd_dyn.length_jac) .+ offset_jac
        evalResult.jac[ind_jac] = nonzeros(problem.jacdata_fwd_dyn.jac)
    end

    return 0
end
