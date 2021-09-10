
A homemade mecanum wheels based robot car, running Robot OS on a Raspberry Pi, with an additional STM32 board, dedicated to motors & making possible to drive the car with a PS2 controller.

This repository gathers the Raspberry Pi files, a.k.a. the ROS car application (RCAP).

## Update Ubuntu

- sudo apt-get update
- sudo apt-get upgrade
- sudo apt-get install vim

## Setup SSH

- sudo apt-get openssh-server
- sudo ufw allow ssh

## Setup VNC

- sudo apt-get install tightvncserver
- sudo vim /etc/systemd/system/tightvncserver.service

>  [Unit]
>  Description=TightVNC remote desktop server  
>  After=sshd.service
>  
>  [Service]
>  Type=dbus
>  ExecStart=/usr/bin/tightvncserver :1
>  User=pi
>  Type=forking
>  
>  [Install]
>  WantedBy=multi-user.target

- sudo chown root:root /etc/systemd/system/tightvncserver.service
- sudo chmod 755 /etc/systemd/system/tightvncserver.service
- sudo systemctl start  tightvncserver.service
- sudo systemctl status tightvncserver.service
- sudo systemctl enable tightvncserver.service

##  Setup serial port

- sudo systemctl disable hciuart
- sudo systemctl disable hciuart.service
- sudo systemctl disable bluetooth.service

- sudo systemctl stop    serial-getty@ttyS0.service
- sudo systemctl disable serial-getty@ttyS0.service
- sudo systemctl mask    serial-getty@ttyS0.service

- sudo usermod -a -G dialout pi
- sudo usermod -a -G tty pi

- sudo vim /boot/firmware/usercfg.txt
>  enable_uart=1
>  dtoverlay=disable-bt

- sudo vim /boot/firmware/cmdline.txt
>  ~~console=ttyAMA0,115200~~

- reboot 
