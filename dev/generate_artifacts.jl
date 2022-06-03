using Pkg
Pkg.activate(temp=true)
Pkg.add("Inflate")

using Downloads
using Pkg.PlatformEngines
using SHA

include("./artifacts.jl")

function generate_artifact(name::String, base_url::String, commit_hash::String, artifacts_toml::String)
    # Query the `Artifacts.toml` file for the hash bound to the name `name`
    # (returns `nothing` if no such binding exists)
    hash = artifact_hash(name, artifacts_toml)
    tarball_url = nothing
    tarball_hash = nothing

    # If the name was not bound, or the hash it was bound to does not exist, create it!
    if isnothing(hash) || !artifact_exists(hash)
        # create_artifact() returns the content-hash of the artifact directory once we're finished creating it
        hash = create_artifact() do artifact_dir

            # Download the tarball (`.tar.gz` archive)
            tarball_url = "$(base_url)/archive/$(commit_hash).tar.gz"
            tarball_path = Downloads.download(tarball_url)
            # @show tarball_path

            try
                # Compute the tarball hash, and unpack the tarball
                tarball_hash = bytes2hex(open(SHA.sha256, tarball_path))
                unpack(tarball_path, artifact_dir)
            finally
                rm(tarball_path)
            end
        end

        # Now bind that hash within our `Artifacts.toml`.  `force = true` means that if it already exists,
        # just overwrite with the new content-hash.  Unless the source files change, we do not expect
        # the content hash to change, so this should not cause unnecessary version control churn.
        bind_artifact!(artifacts_toml, name, hash; download_info=[(tarball_url, tarball_hash)], lazy=true, force=true)
    end

    # Get the path of the name dataset, either newly created or previously generated.
    # this should be something like `~/.julia/artifacts/dbd04e28be047a54fbe9bf67e934be5b5e0d357a`
    path = artifact_path(hash)
    println("Path for `$(name)`: $(path)")
end

function main()
    # This is the path to the Artifacts.toml we will manipulate
    artifacts_toml = joinpath(@__DIR__, "..", "Artifacts.toml")

    for (name, (base_url, commit_hash)) in ARTIFACTS
        generate_artifact(name, base_url, commit_hash, artifacts_toml)
    end
end

main()
