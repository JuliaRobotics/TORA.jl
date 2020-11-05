function plot_results(problem::Problem, robot::Robot, x)
    nₓ = robot.n_q + robot.n_v + robot.n_τ  # dimension of each mesh point

    # Helper ranges
    ind_q = hcat([range(1 + (i * nₓ)                        , length=robot.n_q) for i = (1:problem.num_knots    ) .- 1]...)
    ind_v = hcat([range(1 + (i * nₓ) + robot.n_q            , length=robot.n_v) for i = (1:problem.num_knots    ) .- 1]...)
    ind_τ = hcat([range(1 + (i * nₓ) + robot.n_q + robot.n_v, length=robot.n_τ) for i = (1:problem.num_knots - 1) .- 1]...)
    
    ts = range(0, length=problem.num_knots, step=problem.dt)

    plts = plot(ts         , x[ind_q]', lw=2, legend=nothing, xlabel="time [s]", ylabel="position [rad]"),
           plot(ts         , x[ind_v]', lw=2, legend=nothing, xlabel="time [s]", ylabel="velocity [rad/s]"),
           plot(ts[1:end-1], x[ind_τ]', lw=2, legend=nothing, xlabel="time [s]", ylabel="torque [Nm]")

    plot(
        plts...,
        layout=grid(3,1),
        size=(600,3*400),
        left_margin=20px,
    )
end

function plot(solver_log::SolverLog)

    plt_1 = plot(
        title="feasibility", title_location=:left,
        xlabel="iterations",
        ylabel="error",
        yaxis=(:log10, [1e-11, 1e4]),
        legend=nothing,
    )

    plot!(solver_log.abs_feas_error, c=1, lw=2)
    scatter!([length(solver_log.abs_feas_error)], solver_log.abs_feas_error[end:end], c=1, marker=:star4)

    plt_2 = plot(
        title="objective", title_location=:left,
        xlabel="iterations",
        ylabel="cost (1e4)",
        legend=nothing,
    )

    obj_value = solver_log.obj_value / 1e4

    plot!(obj_value, c=1, lw=2)
    scatter!([length(obj_value)], obj_value[end:end], c=1, marker=:star4)

    plot(
        plt_1, plt_2,
        size=(600,300),
        # left_margin=20px,

        fontfamily = "Times",
        guidefontsize = 12,
        legendfontsize = 12,
        tickfontsize = 10,
        titlefontsize = 15,
    )
end

export
    plot_results,
    plot
