# KML Publisher

Node for publishing the robot GPS position and heading to a KML file for presentation by Google Earth Pro.

## How it works

The KML publisher node subscribes to a topic that provides global odometry information to obtain the current heading, and a topic providing the current GPS position.  It then creates a KML file with a Placemark tag that specifies the robot location, and an IconStyle tag that specifies an image to be used to represent the robot and the heading to use when showing the icon.  A 'Network Link' can be created in Google Earth Pro to read that KML file every N seconds and display the position and heading of the robot.  (A network link can also specify a local file to be read.)

Topics used by this node:

 * For heading: /odometry/global - uses pose orientation to calculate a heading in degrees
 * For GPS Position: /gps/filtered - latitude and longitude

These topics are available when using the robot_localization package to fuse GPS data with odometry and translate GPS coords to UTM as described by this page:
 * http://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html


## Running

Run the kml_publisher node:

```
ros2 run kml_publisher publisher --ros-args -p "kml_filepath:=<path to>/kml_robot_pos.kml"
```

The filename and path to the KML file to be generated should be specified when launching.

In Google Earth Pro:
1. Right Click 'My Places'.
2. Select 'Add', 'Network Link'.
3. Specify a Name such as 'robot position'.
4. For the Link, Browse to and then select the file specified when launching the publisher.
5. Click the Refresh tab, and then choose 'Periodically' for when to update, and a refresh period of 1 second.
6. Click Ok.

The robot position should then be shown in Google Earth Pro and updated every second.  (You will need to adjust the current view to the location of the robot.)

## Display Icon

To use a different icon for the robot in Google Earth Pro, specify the full path to the icon file when launching the publisher:

```
ros2 run kml_publisher publisher --ros-args -p "kml_filepath:=<path to>/kml_robot_pos.kml" -p "robot_icon_filepath:=<path to png file>"
```
