# TORA.jl

*Trajectory Optimization for Robot Arms*

|                             **Documentation**                             |                        **Build Status**                         |                            **Coverage**                             |
| :-----------------------------------------------------------------------: | :-------------------------------------------------------------: | :-----------------------------------------------------------------: |
| [![][docs-stable-img]][docs-stable-url] [![][docs-dev-img]][docs-dev-url] | [![][travis-img]][travis-url] [![][appveyor-img]][appveyor-url] | [![][codecov-img]][codecov-url] [![][coveralls-img]][coveralls-url] |

## Installation

The package can be installed with the Julia package manager.

From the Julia REPL, type `]` to enter the Pkg REPL mode and run:
```
pkg> add https://github.com/ferrolho/TORA.jl
```

Or, equivalently, via the Pkg API:
```
julia> import Pkg; Pkg.add("https://github.com/ferrolho/TORA.jl")
```

## Documentation

- [**STABLE**][docs-stable-url] &mdash; **documentation of the most recently tagged version.**
- [**DEVEL**][docs-dev-url] &mdash; *documentation of the in-development version.*

## Contributions and Questions

Contributions are very welcome, as are feature requests and suggestions.

Please open an [issue][issues-url] if you encounter any problems.

[docs-dev-img]: https://img.shields.io/badge/docs-dev-blue.svg
[docs-dev-url]: https://ferrolho.github.io/TORA.jl/dev

[docs-stable-img]: https://img.shields.io/badge/docs-stable-blue.svg
[docs-stable-url]: https://ferrolho.github.io/TORA.jl/stable

[travis-img]: https://travis-ci.com/ferrolho/TORA.jl.svg?branch=main&token=wa8UTQ2MKiuHJN6QRxtH
[travis-url]: https://travis-ci.com/ferrolho/TORA.jl

[appveyor-img]: https://ci.appveyor.com/api/projects/status/hxhsgmjeloa2rei6?svg=true
[appveyor-url]: https://ci.appveyor.com/project/ferrolho/tora-jl

[codecov-img]: https://codecov.io/gh/ferrolho/TORA.jl/branch/main/graph/badge.svg?token=7KDVBWH74I
[codecov-url]: https://codecov.io/gh/ferrolho/TORA.jl

[coveralls-img]: https://coveralls.io/repos/github/ferrolho/TORA.jl/badge.svg?branch=main
[coveralls-url]: https://coveralls.io/github/ferrolho/TORA.jl?branch=main

[issues-url]: https://github.com/ferrolho/TORA.jl/issues
