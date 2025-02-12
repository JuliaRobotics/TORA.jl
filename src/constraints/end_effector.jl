function ee_position!(position, robot, q::AbstractVector{T}) where {T}
    state = robot.statecache[T]
    set_configuration!(state, q)

    point = Point3D(robot.frame_ee, 0.0, 0.0, 0.0)
    position[:] = transform(state, point, root_frame(robot.mechanism)).v
end

function cb_eval_fc_con_ee(kc, cb, evalRequest, evalResult, userParams)
    x = evalRequest.x
    problem, robot = userParams

    # dimension of each mesh point
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    length_c = 3

    for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
        ind_qᵢ = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
        ind_c = (1:length_c) .+ ((i - 1) * length_c)
        @views ee_position!(evalResult.c[ind_c], robot, x[ind_qᵢ])  # constraint evaluation at that point
    end

    return 0
end

function cb_eval_ga_con_ee(kc, cb, evalRequest, evalResult, userParams)
    x = evalRequest.x
    problem, robot = userParams

    # dimension of each mesh point
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    for (i, k) = enumerate(sort(collect(keys(problem.ee_pos))))
        ind_qᵢ = range(1 + (k - 1) * nₓ, length=robot.n_q)  # indices of the decision variables
        problem.jacdata_ee_position(x[ind_qᵢ])              # jacobian evaluation at that point
        ind_jac = (1:problem.jacdata_ee_position.jac_length) .+ ((i - 1) * problem.jacdata_ee_position.jac_length)
        evalResult.jac[ind_jac] = nonzeros(problem.jacdata_ee_position.jac)  # pass jacobian to Knitro
    end

    return 0
end

function get_ee_path(problem::Problem, robot::Robot, x)
    positions = zeros(3, problem.num_knots)

    # dimension of each mesh point
    nₓ = robot.n_q + robot.n_v + robot.n_τ

    length_c = 3

    for i = (1:problem.num_knots) .- 1
        ind_c = (1:length_c) .+ (i * length_c)
        ind_qᵢ = range(1 + i * nₓ, length=robot.n_q)
        @views ee_position!(positions[ind_c], robot, x[ind_qᵢ])
    end

    return positions
end

function show_ee_path(vis::Visualizer, ee_positions::Matrix{Float64})
    points = collect(eachcol(ee_positions))
    material = LineBasicMaterial(color=colorant"yellow", linewidth=2.0)
    setobject!(vis["ee path"], Object(PointCloud(points), material, "Line"))
    nothing
end

function body_orientation!(dx, robot, x::AbstractVector{T}) where {T}
    state = robot.statecache[T]
    set_configuration!(state, x)

    body_tf = transform_to_root(state, robot.frame_ee)
    mrp = MRP(rotation(body_tf))

    dx[1] = mrp.x
    dx[2] = mrp.y
    dx[3] = mrp.z

    dx
end
