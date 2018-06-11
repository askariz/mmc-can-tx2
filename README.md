### MMC Gimbal Control Demo

#### How to build 
* $ make CC=/opt/gcc-4.8.5-aarch64/install/bin/aarch64-unknown-linux-gnu-gcc 
* copy to bin/mmc_gimbal_ctrl to your TX2
* before run the progam , set up your can device first!
* ./mmc_gimbal_ctrl can0

#### How to setup TX2 Can bus
sudo modprobe can
sudo modprobe mttcan
sudo modprobe can-raw
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
ip -details -statistics link show can0
