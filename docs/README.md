Navigate to the root of the project with
```bash
cd path/to/TORA.jl
```

To build the docs locally run
```bash
julia --color=yes --project=docs docs/make.jl
```

To serve the docs locally run
```bash
julia -e 'using LiveServer; serve(dir="docs/build")'
```

To serve the docs and update them in real time use
```bash
julia --project=docs -ie 'using TORA, LiveServer; servedocs()'
```
