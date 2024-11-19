ENV["GKSwstype"] = "100"  # See https://github.com/jheinen/GR.jl/issues/278

using Documenter, TORA

makedocs(
    modules = [TORA],
    format = Documenter.HTML(),
    warnonly = Documenter.except(),
    sitename = "TORA.jl",
    authors = "Henrique Ferrolho",
    pages = [
        "Home" => "index.md",
        "Getting Started" => [
            "install.md",
            "tutorial.md",
        ],
        "Manual" => [
            "robot.md",
            "problem.md",
            "solver_interfaces.md",
            "solver_log.md",
        ],
        "Advanced" => [
            "new_robot.md",
            "how_it_works.md",
        ]
    ]
)

deploydocs(
    repo = "github.com/JuliaRobotics/TORA.jl",
    devbranch = "main",
)
