#!/usr/bin/env python3

# Import the Main class from the juliacall module
from juliacall import Main as jl

# Import the Pkg module
jl.seval("import Pkg")

# Check the status of the installed packages
jl.seval("Pkg.status()")

# Add package dependencies
jl.seval('Pkg.add("MeshCat")')
jl.seval('Pkg.add("Rotations")')

# Develop the local package of TORA.jl
jl.seval('Pkg.develop(path="../../TORA.jl")')

# Update the package dependencies
jl.seval("Pkg.update()")

# Import the TORA.jl package
jl.seval("using TORA")

# Test the TORA.jl package by calling the greet() function
jl.TORA.greet()
