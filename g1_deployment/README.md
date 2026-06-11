# G1 deployment code 

## Install 

First, clone the repository including submodules.
```bash 
git clone --recursive git@github.gatech.edu:jkamohara3/g1_deployment.git
cd g1_deployment
git clone git@github.com:unitreerobotics/unitree_sdk2.git
```

Then, install Unitree SDK, Mujoco, and deployment code. 

### Unitree SDK
```bash 
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev libglfw3-dev
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install
```

### Unitree mujoco 

First, install mujoco and create symbolic link to its directory. 
```bash 
mkdir ~/.mujoco
cd ~/.mujoco 
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-x86_64.tar.gz
tar -xvzf mujoco-3.3.6-linux-x86_64.tar.gz
```

```bash
cd unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

Then, build unitree mujoco
```bash
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```

### Deployment code
```bash
cd unitree_rl_lab/deploy/robots/g1_29dof # or other robots
mkdir build && cd build
cmake .. && make
```

## Operation 
### Sim2Sim
For this demo, make sure you have XBox or Switch controller. 

```bash 
# terminal1
cd unitree_mujoco/simulate/build
./unitree_mujoco
```

```bash 
# terminal2
cd unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl --network lo --domain_id 1
```

Press L2 + up to enter initial pose. \
Press R1 + X to enter policy mode. \
Enter 9 to disable virtual gantry. 

During policy mode, 
press L2 + A to enter initial pose, 
press L2 + B to enter passive mode. 

During initial pose mode, 
press L2 + B to enter passive mode, 
press R1 + X to enter policy mode. 

Push left stick to forward/backward to control forward velocity
Push left stick to left/right to control lateral velocity 
Push right stick to left/right to control turning velocity

### Sim2Real 

```bash 
cd unitree_rl_lab/deploy/robots/g1_29dof/build
./g1_ctrl --network eth0 --domain_id 0
```