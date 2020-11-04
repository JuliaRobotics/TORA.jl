function plot_results(problem::Problem, robot::Robot, x)
    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh point

    # Helper ranges
    ind_q = hcat([range(1 + (i * nₓ)                        , length=robot.n_q) for i = (1:problem.num_knots    ) .- 1]...)
    ind_v = hcat([range(1 + (i * nₓ) + robot.n_q            , length=robot.n_v) for i = (1:problem.num_knots    ) .- 1]...)
    ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ) for i = (1:problem.num_knots - 1) .- 1]...)
    
    ts = range(0, length=problem.num_knots, step=problem.dt)

    plts = plot(ts         , x[ind_q]', lw=2, legend=nothing, xlabel="time [s]", yaxis="position [rad]"),
           plot(ts         , x[ind_v]', lw=2, legend=nothing, xlabel="time [s]", yaxis="velocity [rad/s]"),
           plot(ts[1:end-1], x[ind_τ]', lw=2, legend=nothing, xlabel="time [s]", yaxis="torque [Nm]")

    plot(
        plts...,
        layout=grid(3,1),
        size=(600,3*400),
        left_margin=20px,
    )
end
