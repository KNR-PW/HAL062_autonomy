# HAL062_autonomy
![OS](https://img.shields.io/badge/OS-Ubutu_24.04-orange)
![ROS](https://img.shields.io/badge/ROS-Lyrical_Luth-cyan)


## Quick Description
HAL062 autonomy software repository. ROS2 based packages, mainly responsible for navigation, perception, mapping, planning and traversal.


## Setup Guide
### 1. General environment setup guide
**Expected result:** Working NVIDIA docker environment.  
Visit our Wiki, and follow instructions on ["How To Setup"](https://wiki.knr.edu.pl/doc/how-to-setup-Ri1gZDAwQM) page.
### 2. Codebase setup guide
**Expected result:** Custom NVIDIA docker environment based development environment for HAL062.
```bash
git clone git@github.com:KNR-PW/HAL062_autonomy.git
cd HAL062_autonomy
docker build -f docker/Dockerfile -t hal062_autonomy .
```
## Launch guide
Simply run one bash script.
```bash
cd HAL062_autonomy/docker
chmod +x launch_docker.sh
./launch_docker.sh
```
