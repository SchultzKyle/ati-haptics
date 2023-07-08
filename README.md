ati-haptics
----------------

# Overview

A ROS package for reading [ATI](https://www.ati-ia.com/products/ft/sensors.aspx) force/torque sensors and reflecting measurements to [Force Dimension](https://www.forcedimension.com/products) haptics devices. This package was tested using an ATI mini F/T sensor and omega.6 haptics device, but it should work for any ATI and Force Dimension devices.

# Installation procedures
The Python [NetFT API](https://github.com/CameronDevine/NetFT) can be installed with  

```sh
$ pip install NetFT
```

The Force Dimension SDK can be found [here](https://www.forcedimension.com/software/sdk).

# Usage
There are three provided launch files:

<ol>
  <li>ati_sensor.launch</li>
    	- Runs ati_sensor.py<br>  
		- Publishes force reading from ATI in newtons<br>  
		- Contains error handling for dropped data from WiFi router
  <li>ati_haptics.launch</li>
        - Runs ati_haptics.cpp<br>
		- Runs haptics device, subscribes to /ati/forces
  <li>ati_demo.launch</li>
        - Launches both ati_sensor and ati_haptics
</ol>

