# Controller

The hardware of the controller is an Arduino Due. 

To protect the input of the Arduino all +5V inputs have to be restricted to +3,3V to protect the Arduino. The Razorcat demonstrator uses a set of 220 kOhm and 100 kOhm resistors. The wiring of the resistors is documented here:
#TBD

The elevat directory contains the Arduino code for the elevator controller.

The tessy directory contains a TESSY project that is able to do unit tests for the controller code. The source directory contains the Arduino controller code in a raw C form to enable the unit tests.


## Running Headless Tests

In a Cygwin shell or Msys shell with 

```
export PATH=/c/Program\ Files/Razorcat/TESSY_4.3/bin/:$PATH
./executeTestAndReport.sh
```

On Linux with Tessy installed

```
./executeTestAndReport.sh
```
## Running Headless Tests in docker

Razorcat provides an official docker image [razorcat/tessy](https://hub.docker.com/r/razorcat/tessy). 

Running it on Windows with Docker Desktop is as simple as calling. 

```
docker run -t -v "%CD%:/work/" -w /work razorcat/tessy:latest ./executeTestAndReport.sh

````

The requirement is that you started a local FLS server instance on the Windows Docker host. If you have access to a FLS server in your company network you can provide the name of the host via the environement variable `-e RAZORCAT_FLS_HOST=fls.license.server.name `.


On Linux the call would be:

```
docker run -t -e LOCAL_USER_ID=$(id -u) -v "$(pwd):/work/" -w /work -e  RAZORCAT_FLS_HOST=fls.license.server.name razorcat/tessy:latest ./executeTestAndReport.sh
```

Here it is important that you provide the local user id `LOCAL_USER_ID=$(id -u)` otherwise the non root user `tessy` inside your container will not have access to the TESSY project mounted in the container.


### Explanation of parameters

* `-t` Allocates a pseudo-TTY and allows tessycmd to show progress bars on the console
* `-v "%CD%:/work/"` mounts the current director in the container at the path `/work/`. **Windows variant**
* `-v "$(pwd):/work/"` mounts the current director in the container at the path `/work/`. **Unix variant**
* `-e LOCAL_USER_ID=$(id -u)` environemt variable `LOCAL_USER_ID` used by the docker containers user `tessy`. In this way the created files are readable by the user on the docker host
* `-w /work` defines the current directory of the docker command in this case ` ./executeTestAndReport.sh`
