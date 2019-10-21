# System setup
0. make sure you have VRX installed somewhere: https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall Including the location of the file.
0. Get docker: https://docs.docker.com/install/linux/docker-ce/ubuntu/ (note: its not as easy as sudo apt install docker!)
1. Make yourself part of the docker group: `sudo usermod -a -G docker $USER`
2. Logout and login so that it updates your groups. type in `groups` to confirm you are in the docker group.
3. hg clone this: https://bitbucket.org/osrf/vrx-docker/src/default/
4. Follow `Submitting from scratch` or `submitting from tisbutascratch` (below)
5. follow these instructions: https://bitbucket.org/osrf/vrx-docker/src/default/ from 'Quick Start Instructions For a Single Trial -  Setting up workspace to run automated evaluation'.
6. A sample fileset is in `vrx_tasks/usyd_Team`. You can just copy the entire folder into `vrx-docker/team_config`.
7. Everyting will take AGES. this is normal. (there are 919 get's if you want some sort of progress indicator, you'll get it when you see it)
8. Before running, goto `vrx-docker/task_config` and uncomment the data from the trials you are trying to run.
9. Login for tisbutascratch is u:`tisbutascratch` pw: `PGjpFUjUfE3RBd9`
10. To debug: You can find the ros-master-uri (i think by default `ROS_MASTER_URI=http://172.16.0.22:11311`) by inspecting the run command. Then `export ROS_MASTER_URI=http://172.16.0.22:11311` in another terminal, then run `rviz` as normal.

# Submitting from scratch
0. Pull a branch you want to submit into some folder `<USYD VRX>`.
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
3. go to here: https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission
4. Follow steps in `option 1`
5. Go to `testing`.

# Submitting from tisbutascratch
3. run `docker run tisbutascratch/usyd_vrx:v2` for the latest version.
5. Go back to System setup.

# DEBUGGING
2. To check what docker processes are running: `docker ps`
2. ENSURE YOUR ENTRYPOINT IS CORRECT, by running `docker inspect <containername>` and searching for the `entrypoint` datum. It should say `./ros_entrypoint.sh`. If it doesn't, run `docker run ---entrypoint ./ros_entrypoint.sh yourdockerfilename`.


# Updating
0. get docker
1. Get the docker image: `docker run -it bash tisbutascratch/usyd_vrx:v1` (this ensures you run bash instead of just running the launch file)
2. NOTE THAT THIS WILL OVERWRITE THE ENTRYPOINT, SCREWING UP YOUR DOCKER IMAGE. To fix this, follow the step after saving your work.
2. do what you need to do
3. Save your work: first `docker ps` to find your container ID, then `docker commit <containerID> <name_of_your_container>`.
4. Change the entrypoint: run `docker run <yourcontainername> --entrypoint './ros_entrypoint.sh` and then recommit your work. Then use `docker inspect` and check the `entrypoint` datum to ensure that this has worked.

