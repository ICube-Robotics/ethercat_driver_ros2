# EtherCAT Driver ROS2 Docker Containers
Provides a basic preconfigured docker container for development purposes. To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag ethercat_driver:humble --file .docker/Dockerfile .
$ docker run -it --privileged --network=host ethercat_driver:humble
```