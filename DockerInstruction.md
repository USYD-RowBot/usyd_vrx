# Instructions for Running Docker

Install Docker:
https://bitbucket.org/osrf/vrx/wiki/tutorials/installDocker

Install Nvidia Docker (optional):
https://bitbucket.org/osrf/vrx/wiki/tutorials/installNvidiaDocker
Clone the Docker files.

Create a folder for Docker

```
mkdir -p ~/vrx/docker
cd ~/vrx/docker
wget https://bitbucket.org/osrf/vrx/raw/default/docker/run.bash
chmod u+x run.bash
```

Run the docker container
(Only tested for nvidia)

```
run.bash -n usydrobotx/usyd_vrx:v1.2019
```

To get access to the container in a another terminal.
Find the container name by running:\
`docker ps -a`

Then access the container by running:\
`docker exec -it <name> bash`

# Submission Testing:
Follow instruction here:
https://bitbucket.org/osrf/vrx/wiki/Testing%20your%20submission
The team name will be "usydrobotx"
the Dockerhub image is:
```
usydrobotx/usyd_vrx:v1.2019
```
