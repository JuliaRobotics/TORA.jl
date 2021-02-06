struct JacobianData{T1 <: SparseMatrixCSC,T2 <: ForwardColorJacCache}
    f!::Function
    jac::T1
    jac_cache::T2
    length_jac::Int
    sparsity::SparseArrays.SparseMatrixCSC{Bool,Int64}

    function JacobianData(f!, output, input)
        # TODO Proper way to do it (currently not working)
        # Issue: https://github.com/SciML/SparsityDetection.jl/issues/41
        # sparsity = jacobian_sparsity(f!, output, input)

        # Workaround
        num_samples = 50
        jac_samples = [ForwardDiff.jacobian(f!, rand!(output), rand!(input)) for _ in 1:num_samples]
        sparsity = sparse(sum(jac_samples) .≠ 0)

        jac = convert.(Float64, sparse(sparsity))

        jac_cache = ForwardColorJacCache(f!, input, dx=output,
                                         colorvec=matrix_colors(jac),
                                         sparsity=sparsity)

        length_jac = nnz(jac)

        T1 = typeof(jac)
        T2 = typeof(jac_cache)

        new{T1,T2}(f!, jac, jac_cache, length_jac, sparsity)
    end
end

function (jd::JacobianData)(x)
    forwarddiff_color_jacobian!(jd.jac, jd.f!, x, jd.jac_cache)
end
