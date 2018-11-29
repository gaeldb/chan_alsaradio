# chan_alsaradio

This is a portage of chan_alsaradio for Asterisk 13+

## Prerequisites

Was tested with Rapsbian Stretch and asterisk 13.14
```
sudo update
sudo apt-get install alsa-utils
sudo apt-get install libncurses5-dev uuid-dev libjansson-dev libxml2-dev libsqlite3-dev libasound2-dev
```

## Compile asterisk-13 with chan_alsaradio

First download asterisk sources and chan_alsaradio sources (choose your branch)
```
cd /usr/src/
sudo wget http://downloads.asterisk.org/pub/telephony/asterisk/releases/asterisk-13.14.1.tar.gz
sudo tar xzfv asterisk-13.14.1.tar.gz

Get the branch you want
sudo wget https://github.com/gaeldb/chan_alsaradio/archive/master.zip
sudo unzip master.zip
```

Let's patch with chan_alsaradio sources and compile
```
cd asterisk-13.14.1
cp ../chan_alsaradio-master/chan_alsaradio.c channels/
echo "chan_alsaradio.so: -lasound" >> channels/Makefile
sudo ./configure
sudo make menuselect
sudo make
sudo make install
```

## Configure

#### Get you USB Alsa sound card device name
plughw:0,0 will work for most USB sound cards.
```
cat /proc/asound/cards
aplay -l
arecord -l
```

#### Configure IAX in asterisk (to communicate with your IAX client, here iaxRPT: see below)
/etc/asterisk/iax.conf
```
[iaxrpt]
type=user
context=airlink
auth=md5
secret=whatever
host=dynamic
disallow=all
allow=gsm,ulaw
notransfer=yes
; Allow Asterisk to handle call from this user without IP whitelist = UNSECURE
requirecalltoken=no
```
Warning: this is unsecure, not for production mode !

/etc/asterisk/extensions.conf
```
;extensions.conf
[globals]
; END [globals]

[general]
autofallthrough=no
; END [general]

[airlink]
exten => radio1,1,NoOp(Start of radio context)
 same => n,Dial(alsaradio/icomvhf)
 same => n,Hangup()
 
exten => h,1,NoOp(End of radio context)
exten => e,1,NoOp(Error or timeout in radio context)
exten => e,n,Hangup()

; END [airlink]
```
Reload IAX module and dialplan in asterisk.

#### Configure your radio/repeater in asterisk (to communicate with your air environnement)
/etc/asterisk/alsaradio.con (use provided default file) and personnalize for your radio
```
[icomvhf]

input_device=plughw:0,0		; ALSA sound input channel
output_device=plughw:0,0	; ALSA sound output channel
serial_device=/dev/ttyS0	; Serial port for control
serial_disable=0          ; 1 to disable serial control (PTT, PCCMDV2, ...)
```

#### Want to autostart asterisk ?
```
sudo make config
sudo systemctl enable asterisk
```

## Test with IAXRpt

You can download iaxRpt to test your Asterisk airlink gateway.
http://www.xelatec.com/xipar/iaxrpt
user: iaxrpt
password: whatever
context: radio1
