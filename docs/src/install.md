# Installation

First things first: install [Julia](https://julialang.org/). You can follow my tutorial [here](https://ferrolho.github.io/blog/2019-01-26/how-to-install-julia-on-ubuntu).

To install TORA.jl, start Julia and enter `Pkg` mode by pressing `]`, and then run
```
add https://github.com/ferrolho/TORA.jl
```

After the command above, you are pretty much done.

!!! tip "You are good to go"

    I recommend you to go and follow the [tutorial](@ref Tutorial) now.

The installation notes below need not be followed right now.
At the end of the tutorial, I will refer you back here.


## HSL Routines for Ipopt

This section will guide you through the steps required to install the [Harwell Subroutine Library (HSL)](http://www.hsl.rl.ac.uk/ipopt/).

!!! info "Information about Ipopt and the HSL"
    TORA.jl uses [Ipopt](https://github.com/coin-or/Ipopt) (**I**nterior-**P**oint **OPT**imizer) by default, a large-scale nonlinear optimisation solver.

    Ipopt itself depends on other solvers to handle systems of linear equations.

    The [Harwell Subroutine Library (HSL)](http://www.hsl.rl.ac.uk/ipopt/) provides a number of linear solvers that can be used in Ipopt. 

!!! tip "Choose a good linear solver"
    Picking a good linear solver is **extremely important** to maximise the performance of nonlinear solvers.

    For example, the linear solver `MA27` is out dated and can be quite slow. `MA57` is a much better alternative, especially for highly-sparse problems (such as trajectory optimisation problem).

First, install the following dependencies:
```
sudo apt install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
```

Clone Ipopt to your `Home` folder:
```
git clone https://github.com/coin-or/Ipopt.git ~/Ipopt
```

Clone the COIN-OR Tools project [ThirdParty-HSL](https://github.com/coin-or-tools/ThirdParty-HSL) into Ipopt's folder:
```
cd ~/Ipopt
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
```

Go to [http://www.hsl.rl.ac.uk/ipopt/](http://www.hsl.rl.ac.uk/ipopt/) and download **Coin-HSL Full (Stable)** [Linux x86_64].

!!! warning
    Downloading **Coin-HSL Full** requires a licence. If you are an academic, you can get one for free.

Extract and rename the downloaded archive into `~/Ipopt/ThirdParty-HSL`:
```
cd ~/Ipopt/ThirdParty-HSL
tar -xvzf ~/Downloads/coinhsl-2019.05.21.tar.gz
mv coinhsl-2019.05.21 coinhsl
```

Configure, make, and install HSL:
```
./configure
make
sudo make install
```

For compatibility, create `libhsl.so` linking to `libcoinhsl.so`:
```
cd /usr/local/lib
sudo ln -s libcoinhsl.so libhsl.so
```

Finally, add the following line to the end of your `.bashrc` (or `.zshrc`):
```
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```

## Artelys Knitro

!!! unknown "Not ready yet"
    Documentation to be written.
