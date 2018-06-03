Setup (Windows):

In order to get started, we need to make sure that we have the correct tools to get the job done. [TODO: Expand the documentation for complete beginners and contract the documentation for advanced users]

1) Download the Raspbian image from here:
https://www.raspberrypi.org/downloads/raspbian/
2) Download Win32 Disk Imager from here:
https://sourceforge.net/projects/win32diskimager/
3) (Optional) Download SD Formatter from here:
https://www.sdcard.org/downloads/formatter_4/
4) Download PuTTY here:
https://putty.org/

4) Insert SD (or microSD) card into computer and open Win32 Disk Imager
5) In the "Image File" box, select the raspian-stretch.img file.
6) Make sure that the appropriate letter drive is selected under "Device" (Example: F:\)
7) Click on the "Write" button.  With my class 10 microSD card, it took about 5 minutes to complete.
8) Unplug the SD card from your computer and insert it into the Raspberry pi.
9) Make sure that a keyboard and mouse is plugged into the Raspberry pi then plug a micro USB cable into it to turn it on.
10) Once the Raspberry pi is booted up, you should see the GUI.  Select the command prompt icon (Terminal) from the top of the screen.  Once the Command Line Interface appears, type "sudo raspi-config".
11) You'll see a blue/grey dialog appear.  Use the arrow keys to navigate to "Boot Options" then press enter.
12) Select "B1 Desktop / CLI" and hit enter, then select "B2 Console Autologin" and hit enter.
13) The blue/grey config screen should still be visible.  Scroll down to "Network Options" and hit enter.
14) Select "Wi-fi", and follow the prompts to connect your Raspberry pi to your Wi-fi.

[TODO: Expand other things which need to be done: keyboard layout, locale, etc.]
- Configure SSH ( and run "sudo /etc/init.d/ssh start")
- run "sudo shutdown -r now". It can take a few minutes before the machine boots back up.
- Set up WiFi here: https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md
- Update raspi-config
- Router DHCP configuration
- Connect to the Raspberry pi with PuTTY
- run "df -h" to see how much space is available
- curl http://www.google.com
- run "sudo apt-get update"
- run "sudo apt-get upgrade" (More info here: https://askubuntu.com/questions/94102/what-is-the-difference-between-apt-get-update-and-upgrade#94104)
- Python3 should already be installed.  Confirm by typing "python3"
- Follow these steps to build Opencv, but stop at the part regarding virtual environments:
https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/
