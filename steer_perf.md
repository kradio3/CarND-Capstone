# Steering Controller Performance

## Measuring the car's position MSE

The `dbw_node_perf` node located in the `ros/src/twist_controller/dbw_node_perf.py` measures the mean squared error (MSE) of the car's position.

The main launch file `launch/styx.launch` has been modified to launch the `dbw_node_perf` node. So no manual work required. Just launch ROS along with the simulator as usually.

The node aggregates the MSE error and prints an info log message to stdout once measuring finished.
Quite ofter the node stops before the message gets printed so see log files to find the message.

## Tuning Log

Using the `dbw_node_perf` to calculate MSE of the car's position.

### 2019-02-16

Track: Highway
Number of samples: 240

**The MSE error when don't adjust `target_angular_vel` in the yaw controller:**

```
Steering performance based on 240 samples = 0.0872404958333297
Steering performance based on 240 samples = 0.0810669833333386
```

**The MSE error of the walkthrough code with updated waypoint follower:**

```
Steering performance based on 240 samples = 0.0828155124999976
Steering performance based on 240 samples = 0.0769877833333294
```

**The MSE error of the walkthough code:**

```
Steering performance based on 240 samples = 0.1451740916666643
```

## Running Performance Test (obsolete)

> this doesn't really help to measure the steering performance. Please use the `dbw_node_perf` node instead.

Run the following command to prepare the test data:

```
make test-init
```

Once done build and run the docker container

```
make build run
```

In the running container start the test node:

```
roslaunch src/twist_controller/launch/dbw_test.launch
```

You may see errors like this:

```
[ERROR] [1550117114.932990800]: Client [/dbw_test] wants topic /actual/brake_cmd to have datatype/md5s│Removing: /Users/nyukhalov/Library/Logs/Homebrew/zsh... (64B)
um [dbw_mkz_msgs/BrakeCmd/899b0f3ef31bf0a48497d65b424a1975], but our version has [dbw_mkz_msgs/BrakeCm│Removing: /Users/nyukhalov/Library/Logs/Homebrew/theora... (64B)
d/c0d20e1056976680942e85ab0959826c]. Dropping connection.
```

The reason of the errors is unknown yet, but looks like we can simply ignore them.

Wait until test node is finished. The following log message indicates that the node finished:

```
[dbw_test-3] process has finished cleanly
```

Now you can exit from the container, we don't need it anymore.

Run the `steering_perf.py` script from the util directory to calculate overall error:

```
python util/steering_perf.py
```

The output will look like this:

```
Steering performance based on 1000 samples = 0.0782663364056498
```