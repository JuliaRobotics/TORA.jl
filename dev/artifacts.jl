using Pkg.Artifacts

const ARTIFACTS = Dict(
    "iiwa_stack" => (
        base_url="https://github.com/IFL-CAMP/iiwa_stack",
        commit_hash="a4f132ad093d276e63040ddbbac6b2cd1facbdcc",
    ),
    "kinova-ros" => (
        base_url="https://github.com/Kinovarobotics/kinova-ros",
        commit_hash="93f66822ec073827eac5de59ebc2a3c5f06bf17f",
    ),
    "ros_kortex" => (
        base_url="https://github.com/Kinovarobotics/ros_kortex",
        commit_hash="7ba75b3e6293fe4af9fe64f35e877d1ff2613360",
    ),
)

artifact_commit_hash(name::String) = ARTIFACTS[name].commit_hash
