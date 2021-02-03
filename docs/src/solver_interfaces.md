# Solver Interfaces

[`Problem`](@ref)s created with TORA.jl are first converted into an [NLP](https://en.wikipedia.org/wiki/Nonlinear_programming) problem, and then solved using [Ipopt](https://github.com/coin-or/Ipopt) or [Knitro](https://www.artelys.com/solvers/knitro/).
The two methods available for this are:
```@index
Pages = ["solver_interfaces.md"]
```
These methods have different names but a very similar signature, purpose, and functionality.

## Arguments

The main arguments of the `solve_with_*` methods are a [`Problem`](@ref) and a [`Robot`](@ref). Moreover, there are optional keyword arguments that can be passed to these methods in order to customize the NLP problem transcribed from the high-level description [`Problem`](@ref). These optional arguments are listed and explained below.

### Initial Guess

`initial_guess=Float64[]`

This is the starting point for the solver. In other words, this is the value estimation of each decision variable. Providing a good initial guess is very important for the solver to converge quickly and in as few iterations as possible. An example of passing an initial guess is shown in [Tutorial](@ref) > [Providing an Initial Guess](@ref).

The *default value* for the `initial_guess` is an empty array, i.e., `Float64[]`. During transcription, TORA.jl checks the value of `initial_guess`. If an array has been provided, it is passed to the solver; if a guess has not been provided, an array filled with zeros is passed to the solver instead.

### Dynamics Defects

`use_inv_dyn=false`

TORA.jl transcribes a [`Problem`](@ref) into an NLP problem using an approach called *Direct Transcription*.[^1]
This approach discretizes the continuous optimization problem in time, and requires nonlinear equality constraints to enforce the full dynamics of the system—known as *dynamics defects*.
There are two options available to define these constraints: using *forward dynamics* or *inverse dynamics*.
For more details, please read [How TORA.jl Works](@ref).

The *default value* for `use_inv_dyn` is `false`. I.e., the default behaviour is to use forward dynamics. In order to use inverse dynamics for defining the dynamics defects, `use_inv_dyn` should be set to `true`.

[^1]:
    Betts, John T. [*Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*](https://epubs.siam.org/doi/book/10.1137/1.9780898718577). SIAM, 2010.

### Energy Minimization

`minimise_τ=false`

By default, problems formulated with TORA.jl are *feasibility problems*.
In other words, the goal for the solver is to find a set of values that satisfies all the constraints of a [`Problem`](@ref).
However, it is possible to define a cost function to be minimized. (Or a value function to be maximized.)

The *default value* for `minimise_τ` is `false`. I.e., the default behaviour is to solve the feasibility problem, without optimizing any objective function. Alternatively, if `minimise_τ` is set to `true`, TORA.jl will minimize the joint torques required by the trajectory being computed.

!!! warning
    Optimizing an objective function is very costly, and may take a significant amount of iterations depending on the [Initial Guess](@ref) provided to the solver.

!!! note
    Currently, TORA.jl provides only one cost function: minimization of torques.
    Different cost functions can be used, but they need to be specified by the user manually.
    Feel free to open a [New Issue](https://github.com/JuliaRobotics/TORA.jl/issues/new) on GitHub if you need some pointers on how and where to get started.

### Solver User Options

`user_options=Dict()`

This parameter allows to pass custom options to the actual NLP solvers, i.e., to [Ipopt](https://github.com/coin-or/Ipopt) and to [Knitro](https://www.artelys.com/solvers/knitro/).
The options that can be set depend on each solver. The list of available options for Ipopt is given [here](https://coin-or.github.io/Ipopt/OPTIONS.html#OPTIONS_REF), and for Knitro [here](https://www.artelys.com/docs/knitro/3_referenceManual/userOptions.html).

The *default value* for `user_options` is an empty dictionary.
A dictionary of key-value pairs can be passed to the solver instead.

As an example, suppose we want to use the linear solver `MA57`. Then, for Ipopt we would set
```julia
user_options=Dict("linear_solver" => "ma57")  # for Ipopt
```
and for Knitro
```julia
user_options=Dict(KNITRO.KN_PARAM_LINSOLVER => KNITRO.KN_LINSOLVER_MA57)   # for Knitro
```

## Methods

```@docs
solve_with_ipopt
solve_with_knitro
```
