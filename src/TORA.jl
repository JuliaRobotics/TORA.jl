module TORA

# Workaround for: https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500
using LinearAlgebra; BLAS.set_num_threads(1)

using ForwardDiff
using GeometryTypes
using KNITRO
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using SparseArrays
using SparseDiffTools

greet() = print("Hello World!")

# `Struct`-defining files
include("./robot.jl")
include("./problem.jl")

# Regular source code
include("./transcription.jl")

end # module
