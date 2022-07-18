@inline function semi_implicit_euler_step!(dx, qᵢ, vᵢ, v̇ᵢ, h, n_q, n_v)
    vᵢ₊₁ = vᵢ .+ h .* v̇ᵢ
    qᵢ₊₁ = qᵢ .+ h .* vᵢ₊₁

    @. dx[(1:n_q)       ] = qᵢ₊₁
    @. dx[(1:n_v) .+ n_q] = vᵢ₊₁

    return dx
end

function forward_dynamics_defects!(defects, robot::Robot{Float64,<:Any,n_q,n_v,n_τ}, x::AbstractVector{T}, h) where {T,n_q,n_v,n_τ}
    result = robot.dynamicsresultcache[T]
    state = robot.statecache[T]

    # Indices of the decision variables
    ind_qᵢ   = (1:robot.n_q)
    ind_vᵢ   = (1:robot.n_v) .+ (robot.n_q)
    ind_τᵢ   = (1:robot.n_τ) .+ (robot.n_q + robot.n_v)
    ind_qᵢ₊₁ = (1:robot.n_q) .+ (robot.n_q + robot.n_v + robot.n_τ)
    ind_vᵢ₊₁ = (1:robot.n_v) .+ (robot.n_q + robot.n_v + robot.n_τ + robot.n_q)

    # Retrieve values of the decision variables
    qᵢ   = SVector{n_q}(@view x[ind_qᵢ  ])
    vᵢ   = SVector{n_v}(@view x[ind_vᵢ  ])
    τᵢ   = SVector{n_τ}(@view x[ind_τᵢ  ])
    qᵢ₊₁ = SVector{n_q}(@view x[ind_qᵢ₊₁])
    vᵢ₊₁ = SVector{n_v}(@view x[ind_vᵢ₊₁])

    # Update robot state
    set_configuration!(state, qᵢ)
    set_velocity!(state, vᵢ)

    # Compute forward dynamics
    dynamics!(result, state, τᵢ)
    v̇ᵢ = SVector{n_v}(result.v̇)

    # Integrate the robot state
    xᵢ₊₁_integrated = @MVector zeros(T, n_q + n_v)
    semi_implicit_euler_step!(xᵢ₊₁_integrated, qᵢ, vᵢ, v̇ᵢ, h, n_q, n_v)

    # Next robot state discretised
    xᵢ₊₁_discretised = [qᵢ₊₁
                        vᵢ₊₁]

    # Evaluate state defects (joint positions and joint velocities)
    @. defects = xᵢ₊₁_integrated - xᵢ₊₁_discretised
end

function inverse_dynamics_defects!(defects, robot::Robot{Float64,<:Any,n_q,n_v,n_τ}, x::AbstractVector{T}, h) where {T,n_q,n_v,n_τ}
    dynamics_result = robot.dynamicsresultcache[T]
    segmented_vector = robot.segmentedvectorcache[T]
    state = robot.statecache[T]

    # Indices of the decision variables
    ind_qᵢ   = (1:robot.n_q)
    ind_vᵢ   = (1:robot.n_v) .+ (robot.n_q)
    ind_τᵢ   = (1:robot.n_τ) .+ (robot.n_q + robot.n_v)
    ind_qᵢ₊₁ = (1:robot.n_q) .+ (robot.n_q + robot.n_v + robot.n_τ)
    ind_vᵢ₊₁ = (1:robot.n_v) .+ (robot.n_q + robot.n_v + robot.n_τ + robot.n_q)

    # Retrieve values of the decision variables
    qᵢ   = SVector{n_q}(@view x[ind_qᵢ  ])
    vᵢ   = SVector{n_v}(@view x[ind_vᵢ  ])
    τᵢ   = SVector{n_τ}(@view x[ind_τᵢ  ])
    qᵢ₊₁ = SVector{n_q}(@view x[ind_qᵢ₊₁])
    vᵢ₊₁ = SVector{n_v}(@view x[ind_vᵢ₊₁])

    # Update robot state
    set_configuration!(state, qᵢ)
    set_velocity!(state, vᵢ)

    # Implicit joint accelerations
    v̇ᵢ = segmented_vector
    copyto!(v̇ᵢ, (vᵢ₊₁ - vᵢ) / h)

    # Compute inverse dynamics
    torques = segmented_vector
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

function make_cb_eval_fc_con_dyn(problem, robot, jacdata, eval_fc!::Function)
    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh points
    length_c = size(jacdata.jac, 1)  # length of constraint per pair of knots

    function cb_eval_fc_con_dyn(kc, cb, evalRequest, evalResult, userParams)
        x = evalRequest.x

        for i = 0:problem.num_knots - 2
            # Calculate the indices of the appropriate decision variables
            ind_vars = range(1 + i * nₓ, length=size(jacdata.jac, 2))

            # Evaluate constraints for Knitro
            offset_con = i * length_c
            ind_con = (1:length_c) .+ offset_con
            @views eval_fc!(evalResult.c[ind_con], robot, x[ind_vars], problem.dt)
        end

        return 0
    end
end

function make_cb_eval_ga_con_dyn(problem, robot, jacdata)
    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh points

    function cb_eval_ga_con_dyn(kc, cb, evalRequest, evalResult, userParams)
        x = evalRequest.x

        for i = 0:problem.num_knots - 2
            # Calculate the indices of the appropriate decision variables
            ind_vars = range(1 + i * nₓ, length=size(jacdata.jac, 2))

            # Evaluate the Jacobian at that point
            jacdata(x[ind_vars])

            # Pass the Jacobian to Knitro
            offset_jac = i * jacdata.length_jac
            ind_jac = (1:jacdata.length_jac) .+ offset_jac
            evalResult.jac[ind_jac] = nonzeros(jacdata.jac)
        end

        return 0
    end
end
