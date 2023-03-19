Authors: Ryan Leahy, Ethan Osgood, and Ethan Higa
Preamble: Currently when this was written on 02/16/2023 the wireless system composed of two TP-link CPE210's. One setup in Access point mode and the other setup in Client mode. The Jetson connects to the client and your computer connects to the access point. 

# Required gear: 

Four ethernet cables 

Two TP-Link CPE210 

Two POE (Power over ethernet) injectors 

A smile 😊 

# CAUTION!!!!!!!! 

WHEN PLUGGING IN THE POE INJECTORS MAKE SURE YOU ARE PLUGGING THE ONE MEANT FOR THE JETSON/COMPUTER INTO THE CORRECT PORT OTHERWISE YOU WILL FRY THEM. 

Luckily for you, as long as they weren't wiped off, the POE injectors should be labeled AP for access point (TP-Link CPE210) and Jetson/Laptop 

 

# Setup: 

1. Grab the TP-Link with the writing on the back that says `Jetson client`. Shove an ethernet cable up its butt, make sure it stays in there and doesn't accidentally come loose. Plug the other end of the ethernet cable into one of the plugs labeled AP on the POE injector with AP and Jetson written on it. Grab another ethernet cable, plug it into the Jetson and then the other end into the POE injector labeled Jetson. Plug the POE injector into power. 

2. Grab the TP-Link with the writing on the back that says `Base station` on it. Do the same as above but instead of jetson related stuff, plug it into your laptop and power on the POE injector. 

3. On your laptop, modify the ethernet settings for the ethernet interface that the wireless link is plugged into to assign a manual ip address to the following. 

  Ip address: `192.168.0.3` 

  Netmask: `255.255.255.0` 

  If subnet prefix length is needed, set to `24` 

  Gateway: `192.168.0.254`  

  If preferred DNS is needed, set to gateway ip as well 

4. Press the power button on the Jetson to turn the jetson on before attempting to connect to it. 
 - If using the Libre, it should be already on when given power and waiting a bit

5. Once the above is done and the wireless links are pointed in each others direction, it should work. 

# To ssh, use the following info below 

Jetson/Libre credentials as of 02/16/2023 

Username: `robotics` 

Password: `robotics2022` 

IP address: `192.168.0.2` 


1. Pull up a terminal of your choosing and type `ssh <Username>@192.168.0.2` 

2. When prompted for a password, enter the above password. 

3. You are now remoted into the Jetson/Libre. 

# Other useful info 

Basestation webserver ip: `192.168.0.254`

Basestation webserver username: `admin` 

Basestation webserver  password: `gurobotics` 

Basestation wifi password: `GURobotics` 

Laptop static ip: `192.168.0.3` 

 

Client webserver ip: `192.168.0.253` 

Client webserver username: `admin` 

Client webserver  password: `gurobotics` 

Jetson static ip: `192.168.0.2` 

 

If for some reason you need to hook your laptop up to the Client TP-Link directly to access the webserver, make sure to change your gateway ip to 192.168.0.253 while using the client and then change it back when you connect to the base station. 

 
