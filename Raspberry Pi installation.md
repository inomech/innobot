<div id="RT_PREEMPT">
ROS 2 Foxy Fitzroy for Ubuntu Focal

# Raspberry Pi and Ubuntu Focal and Real-time (RT_PREEMPT) Kernel 
- Instrukce z [ROS Realtime group github](https://github.com/ros-realtime/linux-real-time-kernel-builder/tree/v5.4.106-rt54-raspi-arm64-lttng-Latest#deploy-new-kernel-on-raspberry-pi4)
  
## Raspberry setup (odzkoušeno na RPi 4) - Ubuntu Server 20.04.05 LTS 64-bit

1) Pomocí RPI Imager naisntalovat na SD kartu (ozkoušeno s 32GB) Ubuntu Server 20.04.05 LTS 64-bit: [Raspberry Pi Generic (64-bit ARM) preinstalled server image](https://cdimage.ubuntu.com/releases/focal/release/ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz) 

2) Po zapsání Ubuntu na SD kartu editni soubor `network-config` a doplň `WIFI-NAME` a `WIFI-PASSWORD`
    - pro headless start (bez klávesnice a myši, jen pomocí ssh):

```
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
 wlan0:
   dhcp4: true
   optional: true
   access-points:
     "WIFI-NAME":
       password: "WIFI-PASSWORD"
```

3) Vysuň SD kartu z PC a vlož do RPi: 

4) Připoj napíjecí kabel a nech inicializovat:

5) cca po 2-3 minutách restartuj RPi (výtahni napájecí kabel a připoj zpátky):
    - RPi se nepodaří napoprvé při inicializace Ubuntu Server připojit na WiFi - proto nutný reset


6) Najdi RPi na síti (z PC - Windows): 

     6.1) naisnatlovat nmap

     6.2) Napiš příkaz `nmap -sn 192.168.1.0/24`

     6.3) Napiš si IP adresu
     - pokud nmap nenašel žádné zařízení potom se RPi nepodařilo připojit na k síti 
     - zkus restartovat RPi (opakuj krok 5)
     - pokud stále nic tak potom byla pravděpodobně chyba ve jménu a heslu k WiFi - opakuj krok 2

7) Připoj se k RPi pomocí ssh z PC - Powershell anebo PuTTy 
    - default přihlašovací údaje pro Ubuntu Server jsou `ubuntu/ubuntu`
    - po prvním připojení se RPi je potřeba změnit heslo - propt je od RPi automaticky
    - po změně hesla tě Ubuntu odpojí - připoj se znovu přes ssh s novým heslem  
8) Update your system. After that you need to connect to the Internet and update your system
```
sudo apt-get update 
sudo apt-get upgrade
sudo reboot
```
9) install Midnight Commander (mc) 
```
sudo apt-get install mc
```
10) Download ready-to-use RT Kernel deb packages from GitHub [Build RT_PREEMPT kernel for Raspberry Pi 4](https://github.com/ros-realtime/linux-real-time-kernel-builder/releases), release for `ubuntu: focal`
```
wget https://github.com/ros-realtime/linux-real-time-kernel-builder/releases/download/5.4.195-rt74-raspi-arm64-lttng/RPI4.RT.Kernel.deb.packages.zip
```
11) Unzip RT Kernel deb packages 
```
unzip RPI4.RT.Kernel.deb.packages.zip
```
12) System information before new kernel inmstalation
```
uname -a
Linux ubuntu 5.4.0-1042-raspi #46-Ubuntu SMP PREEMPT Fri Jul 30 00:35:40 UTC 2021 aarch64 aarch64 aarch64 GNU/Linux
```
13) Install new kernel
```
sudo dpkg -i linux-image-*.deb
sudo reboot
```
14) After reboot you should see a new RT kernel installed and real-time enabled
```
uname -a
Linux ubuntu 5.4.195-rt74-raspi #1 SMP PREEMPT_RT Thu Jul 28 06:58:49 UTC 2022 aarch64 aarch64 aarch64 GNU/Linux

cat /sys/kernel/realtime
1
```
15) Install Ubuntu Desktop 
```
$ sudo apt-get update && sudo apt-get upgrade && sudo apt-get install ubuntu-desktop
```

# ROS2 setup
Next steps according to this instruction : [Install ROS 2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) :
1) Set locale
2) Setup Sources
3) Install ROS 2 packages
4) Environment setup
5) Sourcing the setup script
6) Try some examples
