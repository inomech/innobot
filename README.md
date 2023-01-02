<div align="center">

# INNOBOT

Visit [INNOBOT Home page][def_inobot_homepage]



ROS 2 Foxy | 3D printed | open-source | designed for learning

![](resource/innobot.jpg)


</div>

# Get Started 

## Setup 

To setup your environment follow instructions on [INNOBOT Tutorials](https://www.innobot.eu/docs/tutorials/start)

## Pick and Place in RViz



![](resource/innobot_rviz.PNG)

1. Clone this repository 
2. `cd` into clonned repository
3. If you haven't got ROS 2 Foxy on your machine setup ROS 2 Foxy on your Ubuntu 20.04 machine by executing the helper script `ros-setup.sh`
4. Build using `colcon build` 
5. Start the launch files using the helper script `ros2innobot.bash`
6. Set the position of the object in respect to the robots base in `innobot_core/config/params.yaml`

<div align="center">

![](resource/pnp_params.PNG)

</div>

7. Call the PnP client 
```bash
ros2 run innobot_core pnp_client.py --ros-args --params-file <path to cloned repository>/src/innobot_core/config/params.yaml
```
8. Watch the PnP execution in RViz 

<div align="center">

![](resource/rviz_pnp.gif)

</div>

[def_inobot_homepage]: https://www.innobot.eu