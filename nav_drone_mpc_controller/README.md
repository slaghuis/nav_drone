# MPC Controller
Uses the DLib mpc controller to calculate velocity commands to move the drone 

## Depencencies
### DLib
This algoritm uses the modular predictive controller from the dlib library
```
wget http://dlib.net/files/dlib-19.24.tar.bz2
tar xvf dlib-19.24.tar.bz2
cd dlib-19.24/
mkdir build
cd build
cmake ..
sudo cmake --build . --target install
sudo make install
sudo ldconfig
```

# Code Status
This is still a **work in progress**.  This code has not flown in a simulator yet. (29/09/2022)
