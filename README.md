# TORA.jl
Trajectory Optimisation for Robot Arms

## Install Ipopt

```
sudo apt install gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
```

```
git clone https://github.com/coin-or/Ipopt.git ~/Ipopt
```

To compile the HSL code via the COIN-OR Tools project ThirdParty-HSL, run
```
cd ~/Ipopt
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
```

```
cd ~/Ipopt/ThirdParty-HSL
tar -xvzf ~/Downloads/coinhsl-2019.05.21.tar.gz
mv coinhsl-2019.05.21 coinhsl
```

```
./configure
make
sudo make install
```

Edit your `.bashrc` or `.zshrc` to add:
```
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```

```
cd /usr/local/lib
sudo ln -s libcoinhsl.so libhsl.so
```
