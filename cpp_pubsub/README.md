# cpp_pubsub

Just the ROS2 Tutorials for a minimal publisher and subscriber, but modifed to be in composable node form. 

# Composable Node Launching Weirdness
Running the MinimalPublisher and MinimalSubscriber as components from the command line gives expected behavoir. That would be:
```bash
ros2 run rclcpp_components component_container
```

```bash
ros2 component load /ComponentManager cpp_pubsub MinimalPublisher
ros2 component load /ComponentManager cpp_pubsub MinimalSubscriber
```

However, putting this into a launch file does not give expected behavoir. The terminal output pauses for ~10 seconds before printing ~20 of the Hello World call and returns, then that repeats. It's never the same number of messages that gets printed

```bash
ros2 launch cpp_pubsub pub_sub_comp.launch.py 
```