# This is the code for testing the STAP-PPF method.

## To make the code operational on your machine, do these steps:
1. check the cuda version ```nvcc --version```
2. If installing cuda 11.8 (latest for stable pytorch version):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```
3. If installing cuda 12.1:
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.1.1/local_installers/cuda-repo-ubuntu2004-12-1-local_12.1.1-530.30.02-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-12-1-local_12.1.1-530.30.02-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-12-1-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```
4. Install 11.8 PyTorch:
```
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 
```
or 12.1 Pytorch:
```
pip3 install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu121
```
5. Install 11.8 LibTorch:
```
cd ~/Downloads
curl https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip > libtorch_zip.zip
unzip -o libtorch_zip.zip
sudo mv ~/Downloads/libtorch /usr/lib/libtorch
```
or 12.1 LibTorch:
```
cd ~/Downloads
curl https://download.pytorch.org/libtorch/nightly/cu121/libtorch-cxx11-abi-shared-with-deps-latest.zip > libtorch_zip.zip
unzip -o libtorch_zip.zip
sudo mv ~/Downloads/libtorch /usr/lib/libtorch
```

6. Add some environment variables to bashrc:
```
echo "export PATH=/usr/lib/ccache:/usr/local/cuda/bin:$PATH" >> ~/.bashrc
echo "export CUDACXX=/usr/local/cuda/bin/nvcc" >> ~/.bashrc
echo "export Torch_DIR=/lib/libtorch" >> ~/.bashrc
echo "export CUDA_AVAILABLE=true" >> ~/.bashrc
echo "export OMP_NUM_THREADS=12" >> ~/.bashrc
```

7. Create workspace for Flexible Collision Library
```
mkdir -p ~/jf_fcl_ws/src && cd ~/jf_fcl_ws/src
git clone https://github.com/UF-SAMM-Lab/fcl.git
catkin config --install
catkin build -cs
echo "source /home/$USER/jf_fcl_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

8. Update planning_ws:
```
cd ~/projects/planning_ws/src/ssm_safety
git remote set-url https://github.com/UF-SAMM-Lab/ssm_safety.git
git pull
cd ~/projects/planning_ws/src/motion/human_aware_motion_planners
git remote set-url https://github.com/UF-SAMM-Lab/cari_motion_planning.git
git checkout jf_branch
git pull
cd ~/projects/planning_ws
catkin build -cs
```

9. Update control_ws:
```
cd ~/projects/control_ws/src/motion_controllers/scaled_fjt_controller
git remote set-url origin https://github.com/UF-SAMM-Lab/scaled_follow_joint_trajectory_controller.git
git pull
cd ~/projects/control_ws
catkin build -cs
```

10. Update sharework_ws:
```
cd ~/projects/sharework_ws/src/sharework_cell
git remote set-url origin https://jaredtflowers@bitbucket.org/samm-lab/sharework_cell.git
git pull 
git checkout jf_devel
cd ~/projects/sharework_ws
catkin build -cs
```


11. Create a new workspace with these steps:
```
source ~/.bashrc
mkdir -p ~/projects/test_ws_stap/src && cd ~/projects/test_ws_stap/src
git clone https://github.com/UF-SAMM-Lab/human_predicition.git
git clone https://github.com/UF-SAMM-Lab/human_predicition.git
git clone https://github.com/UF-SAMM-Lab/robot_human_distance_checker.git
git clone https://github.com/UF-SAMM-Lab/STAP_warp.git
cd ~/projects/test_ws_stap
catkin build -cs
```
