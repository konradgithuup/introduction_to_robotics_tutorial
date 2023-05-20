`ros2 launch reactive_behavior robot.launch.py n_robots:=1`

score: 38.07 at t=240.0s

---

`ros2 launch reactive_behavior swarmlab.launch.py n_robots:=1`

score: 19.01 at t=148.5s

The robot clipped a nearby corner at t=148.5s. In compliance with the last rule (don't crash) points accumulated after the collision were disregarded.
