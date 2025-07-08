# ROS_MONITOR
ros_monitor is inspired by [cyber_monitor](https://cyber-rt.readthedocs.io/en/latest/). It is a tui which shows ros topics with their types as well as any messages on the topics. This way one wouldnt have to remember any of the ros cli commands, and have all information in one cli-tool.

![Demo](images/ros_monitor_demo.gif)

TODO:
- [ ] Add more unit tests, use textuals framework
- [ ] Add command line parsing, maybe you only need to inspect 1 topic or a group of specific topics
- [ ] Add configs, right now the topics update at 1hz and the echo updates at 10hz

