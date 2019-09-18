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
12. N.B. robot-state-publisher will probably fail. That's ok. the robot-description argument will be presented in the sim.
12. Save your work: first `docker ps` to find your container ID, then `docker commit <containerID> <name_of_your_container>`.

# Testing
0. hg pull this: https://bitbucket.org/osrf/vrx-docker/src/default/
1. Get the name of the image you want to test from dockerhub e.g. tisbutascratch/usyd_vrx:v2
    1. OR Follow `Submitting from scratch` to push to dockerhub
2. Docker run on your local system just to make sure everything is ok.
2. ENSURE YOUR ENTRYPOINT IS CORRECT, by running `docker inspect <containername>` and searching for the `entrypoint` datum. It should say `./ros_entrypoint.sh`. If it doesn't,
2. follow these instructions: https://bitbucket.org/osrf/vrx-docker/src/default/ from 'Quick Start Instructions For a Single Trial'
3. Everyting will take AGES. this is normal

# Updating
0. get docker
1. Get the docker image: `docker run -it --entrypoint bash tisbutascratch/usyd_vrx:v1` (this ensures you run bash instead of just running the launch file)
2. NOTE THAT THIS WILL OVERWRITE THE ENTRYPOINT, SCREWING UP YOUR DOCKER IMAGE. To fix this, follow the step after saving your work.
2. do what you need to do
3. Save your work: first `docker ps` to find your container ID, then `docker commit <containerID> <name_of_your_container>`.
4. Change the entrypoint: run `docker run <yourcontainername> --entrypoint './ros_entrypoint.sh` and then recommit your work. Then use `docker inspect` and check the `entrypoint` datum to ensure that this has worked.

