mutable struct SolverLog
    x::Array{Float64,2}
    abs_feas_error::Array{Float64,1}
    abs_opt_error::Array{Float64,1}
    obj_value::Array{Float64,1}
    fc_evals::Array{Float64,1}
    ga_evals::Array{Float64,1}
    nStatus::Int  # solution status return code

    @doc """
        SolverLog(N::Int)

    Creates an empty NLP solver log.

    # Arguments
    - `N`: the total number of decision variables of the NLP.
    """
    function SolverLog(N)
        new(Array{Float64,2}(undef, N, 0),
            Array{Float64,1}(undef, 0),
            Array{Float64,1}(undef, 0),
            Array{Float64,1}(undef, 0),
            Array{Float64,1}(undef, 0),
            Array{Float64,1}(undef, 0),
            -999)
    end
end

"""
    update!(log::SolverLog,
            x::Array{Float64,2},
            abs_feas_error::Array{Float64,1},
            abs_opt_error::Array{Float64,1},
            obj_value::Array{Float64,1},
            fc_evals::Array{Float64,1},
            ga_evals::Array{Float64,1})

Updates the log.

# Arguments
- `log`: instance of a `SolverLog`.
- `x`: an intermediate iterate point.
- `abs_feas_error`: abs_feas_error at the current point.
- `abs_opt_error`: abs_opt_error at the current point.
- `obj_value`: obj_value at the current point.
- `fc_evals`: the number of function evaluations requested by the NLP solver so far.
- `ga_evals`: the number of gradient evaluations requested by the NLP solver so far.
"""
function update!(log, x, abs_feas_error, abs_opt_error, obj_value, fc_evals, ga_evals)
    log.x = hcat(log.x, x)
    log.abs_feas_error = vcat(log.abs_feas_error, abs_feas_error)
    log.abs_opt_error = vcat(log.abs_opt_error, abs_opt_error)
    log.obj_value = vcat(log.obj_value, obj_value)
    log.fc_evals = vcat(log.fc_evals, fc_evals)
    log.ga_evals = vcat(log.ga_evals, ga_evals)
end

function length(log::SolverLog)
    num_iters = size(log.x, 2)
    @assert all(==(num_iters), length.([log.abs_feas_error, log.abs_opt_error, log.obj_value, log.fc_evals, log.ga_evals]))
    return num_iters
end

function save(log::SolverLog, file::String)
    npzwrite(file, Dict(
        "x" => log.x,
        "abs_feas_error" => log.abs_feas_error,
        "abs_opt_error" => log.abs_opt_error,
        "obj_value" => log.obj_value,
        "fc_evals" => log.fc_evals,
        "ga_evals" => log.ga_evals,
        "nStatus" => log.nStatus,
    ))
end

function load!(log::SolverLog, file::String)
    data = npzread(file)

    log.x = data["x"]
    log.abs_feas_error = data["abs_feas_error"]
    log.abs_opt_error = data["abs_opt_error"]
    log.obj_value = data["obj_value"]
    log.fc_evals = data["fc_evals"]
    log.ga_evals = data["ga_evals"]
    log.nStatus = data["nStatus"]

    return log
end

export
    SolverLog,
    update!,
    length,
    save,
    load!
