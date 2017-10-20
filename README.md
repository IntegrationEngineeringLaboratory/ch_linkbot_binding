# ch_linkbot_binding version 2.x ALPHA

This document describes how to set up the latest and greatest Ch Linkbot
binding for the latest and greatest Daemon/Library/Firmware Linkbot ecosystem.

## 1. Update your Linkbot Hub

The following command should all be executed on your Linkbot-Hub.

First, you will have to point your Linkbot-Hub to the development repositories.
Attach a monitor/keyboard/mouse to your Linkbot Hub, open a terminal, and issue
the following command to begin editing the repository sources file:

    gedit /etc/apt/sources.list

The original file should look something like this:

```
deb http://mirrordirector.raspbian.org/raspbian/ jessie main contrib non-free rpi
deb http://repo.barobo.com jessie main
# deb http://repo.barobo.com barobo-production main
# Uncomment line below then 'apt-get update' to enable 'apt-get source'
#deb-src http://archive.raspbian.org/raspbian/ jessie main contrib non-free rpi
```

Uncomment the third line so that it now looks like this:
```
deb http://mirrordirector.raspbian.org/raspbian/ jessie main contrib non-free rpi
deb http://repo.barobo.com jessie main
deb http://repo.barobo.com barobo-production main
# Uncomment line below then 'apt-get update' to enable 'apt-get source'
#deb-src http://archive.raspbian.org/raspbian/ jessie main contrib non-free rpi
```

Next, update the daemon and firmware

    sudo apt update
    sudo apt install linkbotd linkbot-firmware

## 2. Reflash your hardware

You will need to reflash any hub and robots you wish to control with the new firmware. 

To flash the dongles, simply unplug them from the Linkbot-Hub and then plug them back in.

To flash robots, turn the robot off, connect it to the Linkbot-Hub with a micro-USB cable, and then
turn the robot back on.

## 3. Configure your Mac(s)

The following commands should all be executed on your Mac computers in a
terminal.

### Download and install `liblinkbot2`

    curl -L https://github.com/davidko/liblinkbot2/releases/download/v2.0.0/liblinkbot-2.0.0-X86_64-Darwin.zip > liblinkbot-2.0.0-X86_64-Darwin.zip
    sudo unzip -d /usr/local liblinkbot-2.0.0-X86_64-Darwin.zip

### Download and install the new Ch Linkbot binding

    curl -L https://github.com/IntegrationEngineeringLaboratory/ch_linkbot_binding/releases/download/v2.0.0/chbarobo-2.0.0-Mac-Intel.zip > chbarobo-2.0.0-Mac-Intel.zip
    unzip chbarobo-2.0.0-Mac-Intel.zip
    cd chbarobo-2.0.0-Mac-Intel
    sudo ch pkginstall.ch chbarobo

## 4. Set an environment variable to point to your local Linkbot-Hub

    export LINKBOT_DAEMON_HOSTPORT=linkbot-hub-aabbcc.local:42001

You should replace the text "aabbcc" with your Linkbot-Hub's 6-digit
identifier. Or, the whole text "linkbot-hub-aabbcc.local" can be replaced with
an IP address. The new daemon uses port 42001 by default while it is in alpha
and beta testing, but will eventually move to port 42000 to replace the old
daemon.

## 5. Test the new binding!

Create a file "test.ch" with the contents 

```C++
#include <linkbot.h>
CLinkbotI robot=CLinkbotI("ZRG6"); // Replace "ZRG6" with your robot ID
robot.resetToZero();
robot.move(90, 90, 90);
```

Run the file

    ch ./test.ch
