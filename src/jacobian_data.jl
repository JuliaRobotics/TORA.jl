import SparseArrays
import SparseDiffTools
# import Symbolics

"""
`JacobianData` is a utility strcture to hold Jacobian information associated with a given function.
"""
struct JacobianData{T,F<:Function,C<:SparseDiffTools.ForwardColorJacCache}
    f!::F
    jac::SparseArrays.SparseMatrixCSC{T,Int}
    jac_cache::C
    jac_length::Int
    jac_sparsity::SparseArrays.SparseMatrixCSC{Bool,Int}

    @doc """
        JacobianData(f!, output, input)

    Create a new `JacobianData`.

    # Arguments
    - `f!`: the method associated with this `JacobianData`
    - `output`: input vector to be passed to `f!`
    - `input`: output vector expected from `f!`
    """
    function JacobianData(f!::F, output::Vector{T}, input::Vector{T}) where {T,F<:Function}
        # # Issue: https://github.com/SciML/SparsityDetection.jl/issues/41
        # # Calculate the sparsity pattern of the Jacobian
        # jac_sparsity = Symbolics.jacobian_sparsity(f!, output, input)

        # [HACKY!] Calculate the sparsity pattern of the Jacobian
        num_samples = 50
        jac_samples = [ForwardDiff.jacobian(f!, rand!(output), rand!(input)) for _ in 1:num_samples]
        jac_sparsity = SparseArrays.sparse(sum(jac_samples) .≠ 0)

        # Placeholder for the actual Jacobian
        jac = T.(jac_sparsity)

        # Color the sparse matrix using graphical techniques (colorvec-assisted differentiation is significantly faster)
        colorvec = SparseDiffTools.matrix_colors(jac)

        # Construct the color cache in advance, in order to not compute it each time the Jacobian needs to be evaluated
        jac_cache = SparseDiffTools.ForwardColorJacCache(f!, input, dx=output, colorvec=colorvec, sparsity=jac_sparsity)

        # The length of the Jacobian, i.e., the number of non-zero elements
        jac_length = SparseArrays.nnz(jac)

        C = typeof(jac_cache)

        new{T,F,C}(f!, jac, jac_cache, jac_length, jac_sparsity)
    end
end

"""
Evaluate this `JacobianData`'s function `f!` for point `x` using forward-mode automatic differentiation.

This method's syntax is "special". For more info on Function-like-objects, read the
[docs](https://docs.julialang.org/en/v1/manual/methods/#Function-like-objects).
"""
(jd::JacobianData)(x) = SparseDiffTools.forwarddiff_color_jacobian!(jd.jac, jd.f!, x, jd.jac_cache)
