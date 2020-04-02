# optoforce_can
ROS Package for optofoce force torque sensor with CAN Communication (C++). 

We use kvaser library for CAN communication

The Project is based on ROS Kinetic version.


```

### Setup Kvaser library ###

$ wget –content-disposition “https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest”
$ tar xf linuxcan.tar.gz
$ cd linuxcan
$ make
$ sudo make install
```


```

### How do I run the Project? ###

$ rosrun optoforce optoforce_node
```
