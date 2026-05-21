#!/bin/bash
set -e

# Disable automatic updates
sudo tee /etc/apt/apt.conf.d/20auto-upgrades > /dev/null <<EOF
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
EOF
echo "[1/5] Auto-updates disabled"

# Prevent boot delay when no network
sudo systemctl mask systemd-networkd-wait-online.service
echo "[2/5] systemd-networkd-wait-online masked"

# Disable suspend and hibernation
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
echo "[3/5] Suspend/hibernation disabled"

# Install OpenCR USB udev rules
sudo cp $CONDA_PREFIX/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "[4/5] OpenCR udev rules installed"

# OpenCR firmware flash prereqs
sudo dpkg --add-architecture armhf
sudo apt-get update -qq
sudo apt-get install -y libc6:armhf
echo "[5/5] OpenCR flash prereqs installed"

echo "Done. Reboot the Pi now: sudo reboot"
