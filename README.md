# The micro-ROS-rtt repository

Basic functionality for measuring round-trip-times (RTT) with micro-ROS and ROS 2. **This repository is work-in-progress.** 


## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).


## Usage

This is intended to be used with the "uros_pong_server" running on a Micro-ROS server. Just start the agent first, and then both the server and the "pingpogn" tool. It will print results to stdout.

The results-file can be read into pandas using `pandas.read_csv("<filename>", sep=" ", index_col=0)`. It has what is hopefully a descriptive header.

Details on the measurement approach are explained in [doc/APPROACH.md](doc/APPROACH.md).


## Requirements, how to build, test, install, use, etc.

Clone the repository into a ROS workspace and build it using [colcon](https://colcon.readthedocs.io/).


## License

micro-ROS-rtt is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

The micro-ROS-rtt repository does not contain other third-party software.


## Quality assurance

**This repository is work-in-progress.** While it is unlikely to be harmful, it contains a benchmarking and testing code that has not been fully verified. Hence, we do not take any responsibility for correctness.