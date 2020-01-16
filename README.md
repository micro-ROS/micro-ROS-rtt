# WORK IN PROGRESS

While this repository is unlikely to be harmful, it 
contains a benchmarking and testing code that has not
been fully verified. Hence, we do not take any responsibility
for correctness.

# USAGE

This is intended to be used with the "uros_pong_server" running 
on a Micro-ROS server. Just start the agent first, and then both
the server and the "pingpogn" tool. It will print results to stdout.

The results-file can be read into pandas using
`pandas.read_csv("<filename>", sep=" ", index_col=0)`
It has what is hopefully a descriptive header

