# Not Supported
This is a personal project which is oriented around my goal to eventually build a [chainsaw wielding robot](https://www.chriskohlhardt.com/forest-thinning-robots) using ROS2. At this stage I'm really just learning. I'm happy to help others who are also learning, but I'm not trying to support this as an active project at this point (and may never do so in the future). I'm making this repository public to help others who are also trying to learn. Good luck!

# Prerequisites

- You'll need an SBC (Single Board Computer) for your robot. At the bottom of this page in the Loadout section I detail the SBC I'm using along with other hardware
- This work is known to workon ROS2/Foxy on Ubuntu 20.04. I think it's likely to work with ROS2/Galactic
- You'll be happier if you can run RVIZ2 on your local machine. I've had good luck using [The RobotStack install of ROS2 on my M1 Mac](https://github.com/RoboStack/ros-galactic) and they offer Windows builds too. You may try the official ROS2 version on your local machine as well. 
- The hardware loadout for each Robot will be described below. 
- The instructions below assume you have [SSH Authentication](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) setup with GitHub on your SBC.

# Helpful Documentation

- If you are new to ROS2, you're best off starting with [the official ROS2 tutorials](https://docs.ros.org/en/foxy/Tutorials.html)
- I've also found that the paid lessons on [The Construct](https://www.theconstructsim.com) are helpful too
- [Nav2 Documentation](https://navigation.ros.org) - The [First-Time Robot Setup Guide](https://navigation.ros.org/setup_guides/index.html) is super helpful when learning

# Installation

- [Install ROS2](https://docs.ros.org/en/foxy/Installation.html)
- Create a new ROS2 workspace (it's ok if you replace ~ for another workspace location, but we'll assume ~/treespotte_ws for this document)
  - mkdir -p ~/treespotte_ws/src/
  - cd ~/treespotte_ws/src
- Install other required ROS2 components - Some of these may be redundent
  - sudo apt install ros-foxy-desktop
  - sudo apt install ros-foxy-xacro
  - sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers
  - sudo apt install ros-foxy-tf-transformations
  - sudo apt install python3-colcon-common-extensions
  - sudo apt install ros-foxy-slam-toolbox
  - sudo pip3 install transforms3d
  - sudo pip install -U rosdep
- [RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros/tree/ros2) 
  - Install with "Method 2" for best performance
  - When you get to "Step 3: Install Intel® RealSense™ ROS2 wrapper from Sources" - You should be installing this in the *~/treespotte_ws/src* folder you created above
- Install YDLidar Drivers
  - cd ~treespotte_ws/src
  - git clone git@github.com:yangfuyuan/ydlidar_ros2.git
  - cd ~/treespotte_ws
  - colcon build --symlink-install 
- Install RoboClaw drivers
  - cd ~/treespotte_ws/src
  - git clone git@github.com:KohlhardtC/roboclaw_ros.git
- Install this treespotte code
  - cd ~/treespotte_ws/src
  - git clone git@github.com:KohlhardtC/treespotte.git
- Check dependancies 
  - cd ~/treespotte_ws
  - sudo rosdep init
  - rosdep update
  - **TODO:I'm missing a step I think**

# Build

- cd ~/treespotte_ws
- . /opt/ros/foxy/setup.bash
- colcon build --symlink-install

# Run

- . install/setup.bash (only need to do this once per terminal session)
- **Setup the robot to map:** ros2 launch treespotte ts3.launch.py
- **Turn on navigation:** ros2 launch treespotte treespotte navigation_launch.py
-  

# Tips & Tricks

I found it a bit painful learning ROS2 (Probably because I tend to skim documentation). Here are some tips that I wish were front and center as I was learning

## Tips

- rviz2 has a learning curve. You'll save yourself some hassle if you use the [rviz2 configuration file](treespotte3.rviz) included in this repository
- If you can't see anything in rviz2, it might be because you don't have Global Options -> Fixed Frame set to *base_link*. If you have mapping working, you'll want to change Global Options -> Fixed Frame set to *map*
- Sometimes you just need to shut down rviz2 and all other ros nodes to get things working properly. 
- You need something to give you the map->odom transformation. For Treespotte 3 that thing is 
- You need something to give you the odom->base_link transformation. For Treespotte 3 that thing is the RealSense wrapper but it's actually transforming to `_pose_frame` so you need to add in a static transform from `_pose_frame` to `base_link`. See the examples below to understand what the expected transforms should look like  
- When building a URDF for your robot, positive X is the front of your robot as per [rep 103](https://www.ros.org/reps/rep-0103.html). In RVIZ2 this shows up as a red bar. If your robot seems to be driving the wrong direction, there is a good chance this is the cause. Ask me how I know! ;)
- Units are meters, kg, seconds as per [rep 103](https://www.ros.org/reps/rep-0103.html) but sometimes nanoseconds are used
- You can't have both a joint_state_publisher and a joint_state_publisher_gui running at the same time, otherwise things will go bonkers
- It's a good idea to have a solid understanding regarding which code is respoinsible for which transform. For example, with TS3, the `map->odom` transform is handled by the static_transform_publisher in the ts3.launch.py launch file, the `odom->_pose_frame` transform is handled by realsense2_camera, and the `_pose_frame->base_link` transform is handled by another static_transform_publsiher. Different robots with different sensors will certainly have different transforms. 

Colors of Axis in RVIZ2:
| color | rotation | axis |
|-----|-----|-----|
|Red|Roll|x|
|Green|Pitch|y|
|Yellow|Yaw|z|

## Useful Commands

| command | explanation |
|-------------------------------------------------------------------------------------|------------|
| `colcon build --symlink-install` | This is how you should typically build |
| `colcon build --packages-select $package` | Save yourself time if you're working on one package |
| `check_urdf $filename` | Is your URDF not rendering? You can check if it's valid with this command.  |
| `xacro $filename.xacro` | Is your xacro not working? You can check if it's valid with this command.  |
| `xacro $filename.xacro \| ros2 run gazebo_ros spawn_entity.py -entity a_name -x 0 -y 0 -z 0 -stdin` | Spawn a XACRO into Gazebo |
| `ros2 run tf2_tools view_frames` | This will save frames.pdf to the location you ran the command. Super helpful when debugging the transformation tree. Usually run this on your local computer, not the robot. Open it up with your favorite PDF reader or web browser |
| `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Steer the robot with your keyboard |
| `upower -i /org/freedesktop/UPower/devices/battery_BAT0` | Check battery status |


# Robot Specific Instructions

## Treespotte 3

# Hardware Loadout

I had a hard time finding examples of hardware loadouts when I was building this, so I'm sharing what's worked for me. No affiliate links. 

## Treespotte 3

![Treespotte 3 image](https://images.squarespace-cdn.com/content/v1/5e94fa70eb55292a277cc50a/f9326c01-3478-4c79-aa75-34e213347d10/IMG_0231.jpeg)

- 3D printed base created in Fusion 360 (models availible upon request)
- [LattePanda 864s](https://www.lattepanda.com/products/lattepanda-alpha-864s.html) with a [M.2 Flash Drive](https://www.amazon.com/gp/product/B08GL575DB/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1)
- [A Battery That Works With Latte Panda Alpha](https://www.youtube.com/watch?v=q9cP0_2gwfs) 
- [RoboClaw 2x7A Motor Controller](https://www.basicmicro.com/Roboclaw-2x7A-Motor-Controller_p_55.html)
- [Battery Holder for 2 18650 Batteries](https://www.amazon.com/Ltvystore-Plastic-Battery-Batteries-Container/dp/B08MWMJ179/ref=sr_1_3?crid=3OOQFMPHI0LB6&keywords=2x%2B18650%2Bbattery%2Bholder&qid=1646693678&sprefix=2x18650%2Bbattery%2Bholder%2Caps%2C166&sr=8-3&th=1) - This plugs into the RoboClaw
- [2 x 18650 Batteries](https://www.batteryjunction.com)
- [Intel T265 Tracking Camera](https://www.intelrealsense.com/tracking-camera-t265/) (discontinued)
- [YDLidar G2](https://www.ydlidar.com/products/view/1.html)
- 2 x [12 Volt Brushed Motors With Encoders](https://www.robotshop.com/en/12v-dc-motor-251rpm-encoder.html)
- [Motor Bracket](https://www.robotshop.com/en/pololu-37d-mm-metal-gearmotor-bracket.html)
- 4 x [Wheels](https://www.robotshop.com/en/pololu-wheel-90-10mm-black-pair.html)
- [Wheel Hubs](https://www.robotshop.com/en/pololu-universal-aluminum-6mm-mounting-hubs-4-40.html)
- [Wire](https://www.amazon.com/gp/product/B088KQFHV7/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1)
- Short USB cables for the Lidar and T265
- [Clamping Collar](https://www.gobilda.com/2910-series-aluminum-clamping-collar-6mm-id-x-19mm-od-9mm-length/)
- [Stainless Steel D-Shaft (6mm Diameter, 40mm Length)](https://www.gobilda.com/2101-series-stainless-steel-d-shaft-6mm-diameter-40mm-length/)


# Troubleshooting

| Error | Possible Solution |
|-------------------------|-------------------|
| Error on startup: `Error, cannot retrieve Yd Lidar health code: ffffffff` | Unplug all YDLidar connections and plug them back in. Make sure all connections are good. Reboot the SBC |
| Encoder isn't advancing | Check to make sure wires are connected reliably |
| LattePanda falls asleep | [This solved my problem with LatePanda falling asleep](https://www.unixtutorial.org/disable-sleep-on-ubuntu-server/) |
| T265 not found | If it was working before, sometimes unplugging the USB and plugging it back it helps |
| \[slam_toolbox-1\] \[INFO\] \[slam_toolbox\]: Message Filter dropping message: frame 'laser' at time 1646404216.077 for reason 'Unknown'  | This error message could be a lot better to help new users. I believe it's saying that the odom->base_link transform is missing, therefore there is no laser transform availible, which means nothing will work. For TS3, this happens when something is wrong with the T265 transformation or something else in the transformation tree is missing. See the **Expected Frames** section below for more detail.  |

**Expected Frames**

It's worth mentioning again that you can solve a lot of problems by making sure your transformations are setup right. You can check this with the command:

`ros2 run tf2_tools view_frames`

If you are having trouble with things working or are getting lots of errors, make sure your transformations tree looks something like these
- [Treespotte 3 Expected Frames](extras/ts3-expected-frames.png)
