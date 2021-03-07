#  Blue ROV2 Up<sup>2</sup> Software Package
This package contains information on how to upgrade Blue ROV2 to use UpSquared as companion computer.

## Installing Ubuntu 18.04 on Up<sup>2</sup>
## Installing FLIR Camera Driver
Download Spinnaker SDK containing all the source code and libraries for FLIR camera [Download Spinnaker SDK](https://59ddab2b6876166d.box.lenovo.com/l/u1F4Yu).

Untar the installation package
```shell
tar xvfx spinnaker-version-amd64-pkg.tar.gz
```
Run the installation script
```shell
cd spinnaker-version-amd64
sh install_spinnaker.sh
```
**WARNING: Spinnaker install its own Qt5 which messes up with ROS Qt5.**
Since we do not use Spinnaker visualization, as a quick fix I deleted the Qt5 shipped with spinnaker. 
```console
cd /opt/spinnaker/lib
sudo rm -r libQt5*
```
Confirm visualization in ROS works by running rviz
```console
rviz
```
##
 