# Gilt Data Collection System

This repository includes all the code used for the RGB-D and Turret camera data collection system. Moreover, the necessary setup instructions, required hardware, and software information are also made available.

## Hardware

- High-bandwidth network switch (Gigabit port switch) (each Orbbec camera broadcasts ~380 Mbps @30fps, color 1920x1080 MJPG compressed, and depth 640x576 Y16 raw format)
- High bandwidth NIC card
- Cat6 Ethernet cable
- High bandwidth Network NAS
- Good computing computer with SSD
- Camera sync hub (Optional, if you want to sync multiple Orbbec cameras)
- Turret Camera
- POE switch with enough power supply per port (for Turret camera, the rated power is 15W, for Orbbec, the rated power is 25W)
- RFID reader system
- RFID tag

## Software and Environment

- Ubuntu 22.04 LTS
- ROS2 humble environment
- Python >=3.10
- Cmake
- Docker (optional)

## Getting Started

### Create a workspace

```bash
mkdir dcs_ws
cd dcs_ws
```

### Clone Gitlab Repository

```bash
git clone https://gitlab.msu.edu/bhujelan/gilt_data_collection
cd gilt_data_collection
```

### Create a Python virtual environment and activate

```bash
python3.10 -m venv dcs
source dcs/bin/activate
```

### Before running requirements.txt, set up the environment

#### Orbbec camera environment setup

- Follow [OrbbecSDK_ROS2 installation guide](https://github.com/orbbec/OrbbecSDK_ROS2) except cloning the repository, as you already have a downloaded package.
- Install ROS2 environment using [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Move the camera packages (OrbbecSDK_ROS2 and turret_camera) to the src folder
```bash
mv camera_packages/OrbbecSDK_ROS2 camera_packages/turret_camera src/
```

### Go to `src/` directory

```bash
cd src/
```

### Install deb dependencies

```bash
sudo apt install libgflags-dev nlohmann-json3-dev  \
ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev
```

### Install udev rules

```bash
cd OrbbecSDK_ROS2/orbbec_camera/scripts
```

### Install required packages

```bash
pip install -r requirements.txt
```

### Download OrbbecViewer

Download from [here](https://github.com/orbbec/OrbbecSDK_v2/releases), extract the zip, and launch the viewer to:

- Set up the Orbbec Camera Network IP, Resolutions, and FPS.
- Verify camera connection using the tool.

Edit IP and camera name in:
`src/OrbbecSDK_ROS2/orbbec_camera/launch/femto_net_camera.launch.py`

### Build the packages

```bash
cd ~/dcs_ws/gilt_data_collection
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build specific packages:

```bash
colcon build --packages-select package_name
```

Build with symlink:

```bash
colcon build --symlink-install (--packages-select package_name)
```

### Source installed packages

```bash
source install/setup.bash
```

### Launch Orbbec camera

```bash
ros2 launch orbbec_camera femto_net_camera.launch.py
```

Or pass parameters:

```bash
ros2 launch orbbec_camera femto_net_camera.launch.py color_fps:=15 depth_fps:=15 ir_fps=15
```

Then run recording scripts:

```bash
python3 recording_scripts/pen1_orbbec_record_rfid_based.py
python3 recording_scripts/pen2_orbbec_record_rfid_based.py
```

Or scheduled:

```bash
python3 recording_scripts/pen1_orbbec_record_scheduled.py
python3 recording_scripts/pen2_orbbec_record_scheduled.py
```

### Launching Orbbec camera in sync mode

Follow the instructions provided [Orbbec-setup-for-syncronization](https://www.orbbec.com/docs-general/set-up-cameras-for-external-synchronization_v1-2/) and also for [Femto-mega_synchronization](https://github.com/orbbec/Multi-Device-Synchronization-Example?tab=readme-ov-file#femtomega-multi-device-synchronization-example)


Run the multi camera syncing script
```bash
ros2 launch orbbec_camera femto_mega_multi_camera_synced.launch.py 
```
You can pass the runtime parameters as in femto_net_camera.launch.py command


To verify the camera syncing, run the following script
```bash
ros2 debug_tool/ros2_topic_timestamp_check.py /pen1_camera_1/depth/image_raw /pen1_camera_2/depth/image_raw
```
You can pass as many topics as you want
---

## Turret Camera Setup

- Default IP: `192.168.1.108`
- Complete camera setup via browser (192.168.1.108)
- Change IP and connect via switch/router
- Update `pen1_camera_node.py` and `pen2_camera_node.py` if needed

Default credentials:
- Username: `admin`
- Password: `Pigs3384`
- IPs: `192.168.0.106` to `192.168.0.109`

### Launch Turret camera

```bash
ros2 run turret_camera pen1_camera_node
ros2 run turret_camera pen2_camera_node
```

### Record with:

```bash
python3 recording_scripts/rgb_video_record.py
```

---

## RFID Settings

Create and activate env:

```bash
python3.10 -m venv rfid
source rfid/bin/activate
cd rfid_scripts
pip install -r requirements.txt
python demo-subscriber-final.py
```

- Select `settings.json`
- Current host: `192.168.0.1`, RFID: `192.168.0.2`

### Set cron job for daily upload

```bash
crontab -e
```

Add:

```bash
5 0 * * * /usr/bin/python3 /your/absolute/path/to/msu_rfid_send_incremental_zip.py
```

### Cron for NAS transfer every 30 min

```bash
*/30 * * * * /your/absolute/path/to/file_transfer_nfs.sh
```

---

## File Transfer from Local to NAS

### Install NFS

```bash
sudo apt update
sudo apt install nfs-common
```

### Allow to access NAS shared folder (eg. bags) from the host computer  
Go to NAS (using web browser)->Control Panel->Shared Folder->select the shared folder->click edit->go to NFS Permission tab-> add host computer IP and save


### Mount NFS Share

```bash
sudo mkdir -p /mnt/nfs_share/bags
sudo mount <nas-ip>:<share-path> /mnt/nfs_share/bags
```

Verify:

```bash
ls /mnt/nfs_share/bags
```

### Copy Files

```bash
cp /path/to/local/file /mnt/nfs_share/bags
cp -r /path/to/local/dir /mnt/nfs_share/bags
```

### Automount in `/etc/fstab`

```fstab
<nas-ip>:<share-path> /mnt/nfs_share/bags nfs defaults 0 0
```

Test:

```bash
sudo umount /mnt/nfs_share/bags
sudo mount -a
ls /mnt/nfs_share/bags
```

## Preprocessing of bag files

### Reindexing missed metadata.yaml
If the recorder missed in creating the metadata file, we can use this script, which scans a folder that contains .db3 files and creates a metadata.yaml file if not exist in the bag folder. Usage eg; python reindex_bag.py '<bag_parent_directory>'

```bash
python bag_preprocessing/reindex_bag.py bags/pen1/2D
```

### Reordering ros2 topic timestamps
The network camera topics may arrive at the host computer at different times than it captured, which might mismatch the proper ordering of topic timestamps. This script retrieves the topics header and reorders them in the correct sequence. 

```bash
python bag_preprocessing/reorder_bag_by_header.py bags/pen1/2D
```




