module TORA

# Workaround for: https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500
using LinearAlgebra; BLAS.set_num_threads(1)

using Colors
using ForwardDiff
using GeometryTypes
using Ipopt
using KNITRO
using MeshCat
using MeshCatMechanisms
using Random: rand!
using RigidBodyDynamics
using SparseArrays
using SparseDiffTools

greet() = print("Hello World!")

# `Struct`-defining files
include("./jacobian_data.jl")
include("./robot.jl")
include("./problem.jl")

# Regular source code
include("./transcription/ipopt.jl")
include("./transcription/knitro.jl")

end # module
