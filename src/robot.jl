"""
A `Robot` represents a mechanism, its state, and other relevant objects.

# Relevant fields in `Robot`
- The arrays `q_lo`, `q_hi`, `v_lo`, `v_hi`, `τ_lo`, `τ_hi` store the lower and upper bounds of the joint positions, velocities, and torques, respectively.
- The number of generalized coordinates, generalized velocities, and actuated joints are stored in `n_q`, `n_v`, `n_τ`, respectively.
- The end-effector frame is stored in `frame_ee`.
"""
struct Robot{T,T_SC,n_q,n_v,n_τ}
    urdfpath::String
    mechanism::Mechanism{T}
    state::MechanismState{T}
    statecache::T_SC
    dynamicsresultcache::DynamicsResultCache{T}
    mvis::MechanismVisualizer

    q_lo::Array{T}
    q_hi::Array{T}
    v_lo::Array{T}
    v_hi::Array{T}
    τ_lo::Array{T}
    τ_hi::Array{T}

    n_q::Int64  # Number of generalized coordinates
    n_v::Int64  # Number of generalized velocities
    n_τ::Int64  # Number of actuated joints

    frame_ee::CartesianFrame3D  # End-effector frame

    @doc """
        Robot(urdfpath, mechanism, frame_ee, mvis)

    Create a new `Robot`.
    """
    function Robot(urdfpath::String,
                   mechanism::Mechanism,
                   frame_ee::CartesianFrame3D,
                   mvis::MechanismVisualizer)
        state = MechanismState(mechanism)
        statecache = StateCache(mechanism)
        dynamicsresultcache = DynamicsResultCache(mechanism)

        q_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower, position_bounds(joint))]
        q_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper, position_bounds(joint))]
        v_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower, velocity_bounds(joint))]
        v_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper, velocity_bounds(joint))]
        τ_lo = [lim for joint in joints(mechanism) for lim in map(x -> x.lower,   effort_bounds(joint))]
        τ_hi = [lim for joint in joints(mechanism) for lim in map(x -> x.upper,   effort_bounds(joint))]

        n_q = num_positions(mechanism)
        n_v = num_velocities(mechanism)
        n_τ = num_velocities(mechanism)

        T_SC = typeof(statecache)

        new{Float64,T_SC,n_q,n_v,n_τ}(
            urdfpath,
            mechanism,
            state,
            statecache,
            dynamicsresultcache,
            mvis,
            q_lo,
            q_hi,
            v_lo,
            v_hi,
            τ_lo,
            τ_hi,
            n_q,
            n_v,
            n_τ,
            frame_ee
        )
    end
end

"""
    create_robot_kuka_iiwa_14(vis)

Create a new [KUKA LBR iiwa 14](https://www.kuka.com/en-gb/products/robotics-systems/industrial-robots/lbr-iiwa) robot.
"""
function create_robot_kuka_iiwa_14(vis::Visualizer)
    package_path = joinpath(@__DIR__, "..", "iiwa_stack")
    urdfpath = joinpath(@__DIR__, "..", "robots", "iiwa14.urdf")

    mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
    frame_ee = default_frame(findbody(mechanism, "iiwa_link_pen_tip"))
    remove_fixed_tree_joints!(mechanism)

    urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
    mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
    # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

    Robot(urdfpath, mechanism, frame_ee, mvis)
end

"""
    create_robot_kinova_gen3_lite(vis)

Create a new [Kinova Gen3 lite](https://www.kinovarobotics.com/en/products/gen3-lite-robot) robot.
"""
function create_robot_kinova_gen3_lite(vis::Visualizer)
    package_path = joinpath(@__DIR__, "..", "ros_kortex")
    urdfpath = joinpath(@__DIR__, "..", "robots", "gen3_lite_gen3_lite_2f.urdf")

    mechanism = parse_urdf(urdfpath, remove_fixed_tree_joints=false)
    frame_ee = default_frame(findbody(mechanism, "tool_frame"))
    remove_fixed_tree_joints!(mechanism)

    urdfvisuals = URDFVisuals(urdfpath, package_path=[package_path])
    mvis = MechanismVisualizer(mechanism, urdfvisuals, vis["robot"])
    # setelement!(mvis, frame_ee)  # Visualize a triad at the end-effector

    Robot(urdfpath, mechanism, frame_ee, mvis)
end

export
    Robot,
    create_robot_kuka_iiwa_14,
    create_robot_kinova_gen3_lite
