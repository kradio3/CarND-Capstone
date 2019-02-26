
### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Make commands

1. Build docker container

```bash
make build
```

2. Run docker container

```bash
make run
```

3. Attach to docker container

```bash
make attach
```

4. Prepare workspace for integration test

```bash
make test-init
```

### Team Lead
* [Elias Khsheibun](https://github.com/elias3) - elias3@gmail.com
### Team Members
* [Roman Niukhalov](https://github.com/nyukhalov) - r.nyukhalov@gmail.com
* [Andrey Yaroshenko](https://github.com/kradio3) - kradio3@gmail.com
* [Nathan Sherrit](https://github.com/nathansherrit) - nathansherrit@yahoo.com
* [Ragupathy Gopalakrishnan](https://github.com/ragu-git) - Ragupathy.gopal@gmail.com

### Docs

- [Project instructions by Udacity](UDACITY.md)
- [Steering Controller Performance](steer_perf.md)
