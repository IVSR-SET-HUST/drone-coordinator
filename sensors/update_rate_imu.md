Based on [issue](https://github.com/mavlink/mavros/issues/1346)

Download [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
```
chmod +x ./QGroundControl.AppImage
```
Restart or logout your PC.
```
sudo chmod 666 /dev/ttyACM0 #px4 serial device
./QGroundControl.AppImage  (or double click)
```
Be sure your px4 is connect to PC.

On QGroundControl window, Click Analyze (A sheet with magnifying glass symbol on menu bar) >> MAVLink Console

On MAVLink Console:
```
cd /fs/microsd/etc
echo "mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 100" > extras.txt
```
Note: 
If you want to update exist extras.txt file, you can do:
```
cd /fs/microsd/etc
rm extras.txt
echo "mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 100" > extras.txt
```

