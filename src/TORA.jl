module TORA

using Colors
using ForwardDiff
using GeometryBasics
using Ipopt
using MeshCat
using MeshCatMechanisms
using NPZ
using Plots
using Requires
using RigidBodyDynamics
using SparseArrays
using SparseDiffTools
using StaticArrays

using Plots.PlotMeasures: px
using Random: rand!

import Base: length

"""
    solve_with_knitro(problem, robot;
                      initial_guess=Float64[],
                      use_inv_dyn=false,
                      minimise_τ=false,
                      user_options=Dict())

Solve the nonlinear optimization problem with Knitro.

Further options can be set using the keyword arguments. See [Solver Interfaces](@ref).

# Keyword arguments
- `initial_guess::Vector{Float64}=Float64[]`: the starting point for the solver.
- `use_inv_dyn::Bool=false`: if true, enables the use of inverse dynamics instead of forward dynamics.
- `minimise_τ::Bool=false`: if true, activates a cost function to minimize the joint torques.
- `user_options::Dict=Dict()`: the user options for Knitro.

See also: [`solve_with_ipopt`](@ref)
"""
function solve_with_knitro end

function __init__()
    @require KNITRO = "67920dd8-b58e-52a8-8622-53c4cffbe346" include("./transcription/knitro.jl")
end

greet() = println("Hello World!")

# Artifacts (robot meshes, URDF files, etc.)
include("../dev/artifacts.jl")

# `Struct`-defining files
include("./jacobian_data.jl")
include("./robot.jl")
include("./problem.jl")
include("./solverlog.jl")

# Regular source code
include("./constraints/dynamics.jl")
include("./constraints/end_effector.jl")
include("./transcription/ipopt.jl")
include("./plots.jl")
include("./utils.jl")

# Code to "exercise" the package - see https://julialang.github.io/PrecompileTools.jl/stable/
include("./precompile.jl")

export solve_with_knitro

end # module
