"""
During optimization, a `SolverLog` is used to store convergence information.

A `SolverLog` can log the following:
- `x::Matrix{Float64}`: the history of the decision variables for every iteration.
- `abs_feas_error::Vector{Float64}`: the absolute feasibility error evolution.
- `abs_opt_error::Vector{Float64}`: the absolute optimality error evolution.
- `obj_value::Vector{Float64}`: the evolution of the objective function value.
- `fc_evals::Vector{Float64}`: the evolution of the number of function evaluations.
- `ga_evals::Vector{Float64}`: the evolution of the number of gradient evaluations.
- `nStatus::Int`: the solution status return code.

The actual values stored in a `SolverLog` depend on the implementation of the callback used by each solver for every new point.
Using Ipopt, the callback is set with [`Ipopt.setIntermediateCallback`](https://github.com/jump-dev/Ipopt.jl/blob/d9e9176620a9b527a08991a3d41062fa948867f7/test/C_wrapper.jl#L123-L131).
Using Knitro, the callback is set with [`KNITRO.KN_set_newpt_callback`](https://github.com/jump-dev/KNITRO.jl/blob/43de7d38f01d7aa7c418bb24d0c1ab89b61e527a/examples/nlp2.jl#L201-L204).
"""
mutable struct SolverLog
    x::Matrix{Float64}
    abs_feas_error::Vector{Float64}
    abs_opt_error::Vector{Float64}
    obj_value::Vector{Float64}
    fc_evals::Vector{Float64}
    ga_evals::Vector{Float64}
    nStatus::Int  # solution status return code

    @doc """
        SolverLog(N)

    Create an empty NLP solver log.

    # Arguments
    - `N::Int`: the total number of decision variables of the NLP.

    See also: [`update!`](@ref)
    """
    function SolverLog(N::Int)
        new(Matrix{Float64}(undef, N, 0),
            Vector{Float64}(undef, 0),
            Vector{Float64}(undef, 0),
            Vector{Float64}(undef, 0),
            Vector{Float64}(undef, 0),
            Vector{Float64}(undef, 0),
            -999)
    end
end

"""
    update!(log; <keyword arguments>)

Update a [`SolverLog`](@ref) object.

# Keyword arguments
- `x=zeros(size(log.x, 1))`: an intermediate iterate point.
- `abs_feas_error=0`: the absolute feasibility error at the current point.
- `abs_opt_error=0`: the absolute optimality error at the current point.
- `obj_value=0`: the value of the objective function at the current point.
- `fc_evals=0`: the number of function evaluations requested by the NLP solver so far.
- `ga_evals=0`: the number of gradient evaluations requested by the NLP solver so far.
"""
function update!(log::SolverLog;
                 x=zeros(size(log.x, 1)),
                 abs_feas_error=0,
                 abs_opt_error=0,
                 obj_value=0,
                 fc_evals=0,
                 ga_evals=0)
    log.x = hcat(log.x, x)
    log.abs_feas_error = vcat(log.abs_feas_error, abs_feas_error)
    log.abs_opt_error = vcat(log.abs_opt_error, abs_opt_error)
    log.obj_value = vcat(log.obj_value, obj_value)
    log.fc_evals = vcat(log.fc_evals, fc_evals)
    log.ga_evals = vcat(log.ga_evals, ga_evals)
end

"""
    length(log) -> Integer

Return the length of a [`SolverLog`](@ref).

The length of a [`SolverLog`](@ref) is the same as the number of iterations taken to solve the problem.
"""
function length(log::SolverLog)
    num_iters = size(log.x, 2)
    @assert all(==(num_iters), length.([log.abs_feas_error, log.abs_opt_error, log.obj_value, log.fc_evals, log.ga_evals]))
    return num_iters
end

"""
    save(log, file)

Save a [`SolverLog`](@ref) to an `.npz` file.

See also: [`load!`](@ref)
"""
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

"""
    load!(log, file)

Load an `.npz` file and store it in a [`SolverLog`](@ref) object.

See also: [`save`](@ref)
"""
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
