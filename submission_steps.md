# Submitting from scratch
0. Pull a branch you want to submit into some folder `<USYD VRX>`.
1. go to here: https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission
2. Follow steps in `option 1`
3. run `docker run <name_of_your_container>`
4. run `docker ps` to find your container name under the NAMES Column:
5. open a bash in the docker with `docker exec -it <container_name> bash`
6. Make sure you have sourced the ros melodic files, so you can call catkin_make `source /opt/ros/melodic/setup.bash`
6. create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace 
7. pull our files: `git clone https://github.com/USYD-RowBot/usyd_vrx.git`
8. IF YOU'RE WORKING ON A BRANCH: checkout to and fetch the branch: `git checkout --track origin/stable_steven`
9. Install tf: `apt install ros-melodic-tf` 
10. and also install robot state publisher:`apt install ros-melodic-robot-state-publisher`
10. and also install geographic msgs:`apt install ros-melodic-geographic-msgs`
10. and also install pyproj:`apt install python-pip` then `pip install pyproj`
10. catkin_make and make sure everything is ok.
11. change the run file `~/run_my_system.bash` to whatever script you want to run. An example is included in `vrx_tasks/sampleRunbash.sh`.
12. Make sure it works :3 by running `./run_my_system.bash`.
12. Save your work: first `docker ps` to find your container ID, then `docker commit <containerID> <name_of_your_container>`.


# Testing
0. hg pull this: https://bitbucket.org/osrf/vrx-docker/src/default/
1. Get the name of the image you want to test from dockerhub e.g. tisbutascratch/usyd_vrx:v0
    1. OR Follow `Submitting from scratch` to push to dockerhub
2. follow these instructions: https://bitbucket.org/osrf/vrx-docker/src/default/ from 'Quick Start Instructions For a Single Trial'


# Updating
0. get docker
1. Get the docker image: `docker run -it --entrypoint bash tisbutascratch/usyd_vrx:v1` (this ensures you run bash instead of just running the launch file)
2. do what you need to do
3. Save your work: first `docker ps` to find your container ID, then `docker commit <containerID> <name_of_your_container>`.
