using Documenter, TORA

makedocs(
    modules = [TORA],
    format = Documenter.HTML(),
    sitename = "TORA.jl",
    authors = "Henrique Ferrolho",
    pages = [
        "Home" => "index.md",
        "Getting Started" => [
            "install.md",
            "tutorial.md",
        ],
        "Manual" => [
            "methods.md",
            "advanced.md",
            "new_robot.md",
        ]
    ]
)

deploydocs(
    repo = "github.com/ferrolho/TORA.jl",
    devbranch = "main",
)
