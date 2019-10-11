# KinectCloud

Command line utilities for Azure Kinect DK for generating colored point clouds. Additionally the headers can be used as a wrapper for the Azure Kinect KD to perform the same functionality as the command line.

Example command line usage:

```powershell
# store a point cloud from each connected device
# using depth mode NFOV_2X2BINNED and color resolution 1280x720
KinectCloud.exe -s -da -dma NFOV_2x2BINNED -dra 720P
```

```powershell
# store a point cloud from each connected device
# serial # 000128192012 set to subordinate mode
# serial # 000123492512 set to master mode
# any other connected devices in standalone mode
KinectCloud.exe -s -da -dt 000128192012 s -dt 000123492512 m
```

```powershell
# store a point cloud for the first device with exposure time 31250 us, color balance 4800K
KinectCloud.exe -s -ce 31250 -cw 4800
```

```powershell
# print out help
KinectCloud.exe
```

```
options:
 -s              | capture a colored pointcloud for devices (by default first device only)
 -o path         | specify output locations (default %s_%f.pts)
                 | %s -> serial number, %f -> frame number
 -w int          | wait a number of milliseconds after device startup (if applicable)
 -c int          | store n consecutive frames (if applicable)
 -ds ser         | device operations apply device with given serial number
 -da             | device operations apply to all devices
 -dn             | if no other devices are specified, do not perform any device operations
 -dt ser {a,m,s} | declare device topology, stAndalone, Master or Subordinate
 -dr ser {res}   | declare resolution to use for specific device
 -dra {res}      | declare resolution to use for all devices
   color resolutions: { 720P, 1080P, 1440P, 1536P, 2160P, 3072P }, default is 720P
 -dm ser {mode}  | declare depth mode to use for specific device
 -dra {mode}     | declare depth mode to use for all devices
   depth modes      : { NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED }, default is NFOV_2X2BINNED
 -ce int         | color camera exposure time in nanoseconds for all devices
 -cw int         | color camera white balance in kelvin for all devices (must be % by 10)
 -v              | verbose output
```
