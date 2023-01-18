[TOC]


**ROS 2 Foxy Fitzroy for Ubuntu Focal**

<div id="RT_PREEMPT">

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
  - po isntalaci UBUNTU, pokud je problém se síˇovýcm připojením. Konfigurační soubor pro síťové adaptéry `/etc/netplan/50-cloud-init.yaml`
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            optional: true
            access-points:
                "WIFI-NAME":
                    password: "WIFI-PASSWORD"
            dhcp4: true


```
  - v případě změny konfiguračního souboru aplikovat `netplan` příkazy. Wifi se zprovozní až po resratu. V GUI UBUNTU se nezobrazují sítě protože není instalovný GUI manager. Ubuntu server nemá standartně GUI,
```
sudo netplan generate
sudo netplan try
sudo netplan apply
sudo reboot
ip a
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

## Troubleshooting: 

Pokud výpis z `uname -v` neobsahuje PREEMPT_RT, ale myslíš, že jsi vše udělal OK podle instrukcí, potom RPi ignorovalo nový kernel a je potřeba mu ho vnutit:

Je nutno upravit vmlinuz a initrd.img:

Zadej pojednom tyhle příkazy a doplň instalovanou verzi kernelu:

- v /boot jsou soubory s názvy jako `config-5.4.0-1053-raspi` a `config-5.3.162.rt68` 

  - `5.4.0-162-rt68` odpovídá ``<kernel-version-rt>`` v `ln` příkazech dole   
  - `5.4.0-1053-raspi` odpovídá ``<kernel-version>`` v `ln` příkazech dole 
  - pokud máš novější kernel a v názvy jsou jiné tak v `ln` příkazech zadej svoji verzi

Nacpi RPi nový kernel silou: 

``ubuntu@ubuntu:~$ sudo ln -s -f /boot/vmlinuz-<kernel-version-rt> /boot/vmlinuz``  
``ubuntu@ubuntu:~$ sudo ln -s -f /boot/vmlinuz-<kernel-version> /boot/vmlinuz.old``  
``ubuntu@ubuntu:~$ sudo ln -s -f /boot/initrd.img-<kernel-version-rt> /boot/initrd.img``  
``ubuntu@ubuntu:~$ sudo ln -s -f /boot/initrd.img-<kernel-version> /boot/initrd.img.old``  
``ubuntu@ubuntu:~$ cd /boot``  
``ubuntu@ubuntu:/boot$ sudo cp vmlinuz firmware/vmlinuz``  
``ubuntu@ubuntu:/boot$ sudo cp vmlinuz firmware/vmlinuz.bak``  
``ubuntu@ubuntu:/boot$ sudo cp initrd.img firmware/initrd.img``  
``ubuntu@ubuntu:/boot$ sudo cp initrd.img firmware/initrd.img.bak``  

``sudo reboot``  

Připoj se zpět na RPi a vyzkoušej `uname -v`

</div>

<div id="ROS2">

# ROS2 setup
Next steps according to this instruction : [Install ROS 2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) :
- Set locale
- Setup Sources
- Install ROS 2 packages
- Environment setup
- Sourcing the setup script
- Try some examples

All steps by running script, see step 2.

1) clone git repo z GitLabu branch product-clean do adresáře innobot   
``git clone -b product-clean https//gitlab.com/INOMECH/projekty/247-univerzalni_ridici_jednotka.git ~/innobot``
2) spusť instalaci ROS na RPi   
``cd ~/innobot``  
``./ros-setup.sh``
3) If you always want /opt/ros/foxy/setup.bash sourced when you open a new bash shell, put the source command at the end of the .bashrc file in your home directory.
```
# Set up  environment for ROS2  by sourcing the following file
echo "Enviroment for ROS2 is set. You have access to the ROS commands."
source /opt/ros/foxy/setup.bash
```
4) po dokončení zkontroluj instalaci   
``./check-config-rpi.bash``
5) pokud se instalace nezdaří s výpisem `E: Unable to locate package ***` je problém s [Open Robotics GPG key](https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669)
  -  v příkazovém řádku zadej: 
  ```
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```
6) pokud u některé knihovny/balicku výstup FAIL tak nutno doinstalovat manuálně 
    - ``sudo apt install <debianPackage>``, pokud potřeba instalovat debian package
    - ``pip install <pythonPackage>``, pokud potřeba instalovat python knihovnu
</div>


# Remote desktop connections
Popstupovat podle návodu [Install Xrdp On Ubuntu 20](https://operavps.com/install-xrdp-on-ubuntu-20/), nespouštět firewall
1. Jsou to tyto příkazy:
```
sudo apt-get update -y
sudo apt install xubuntu-desktop
sudo apt install xrdp
sudo systemctl status xrdp
sudo adduser xrdp ssl-cert
sudo systemctl restart xrdp
```
2. Create a file called .xsession in the home folder 
```
echo -e  "startxfce4\n" >> ~/.xsession
chmod +x ~/.xsession
```

## Troubleshooting:
### light-locker 
Po připojení ke vzdálené ploše :
```
ProblemType: Crash
Package: light-locker 1.8.0-1ubuntu1
```
odinstalovat loght-locekr:
```
sudo apt-get remove light-locker
sudo systemctl restart xrdp
```
### Log files
xRDP writes some log files into your system. We would recommend you to have a look at these log files. These logs files might provide useful insight about the problem you are encountering.

You should look at the following files:
```
cat ~/.xsession-errors
cat /var/log/xrdp.log
cat /var/log/xrdp-sesman.log
```

