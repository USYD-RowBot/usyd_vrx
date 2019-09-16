# Submitting from scratch
0. Pull a branch you want to submit into some folder `<USYD VRX>`.
1. go to here: https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission
2. Follow steps in `option 1`
3. run `docker run <name_of_your_container>`
4. run `docker ps` to find your container name under the NAMES Column:
5. open a bash in the docker with `docker exec -it <container_name> bash`
6. create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace (you'll need to source the setup bash because ltr nothing is in bashrc)
7. pull our files: `git clone whatever`
8. IF YOU'RE WORKING ON A BRANCH: checkout to and fetch the branch: `git checkout --track origin/stable_steven`
9. Install tf: `apt install ros-melodic-tf`
10. catkin_make
11. change the run file `run_my_system.bash` to whatever script you want to run.

# Testing
0. hg pull this: https://bitbucket.org/osrf/vrx-docker/src/default/
1. Get the name of the image you want to test from dockerhub e.g. tisbutascratch/usyd_vrx:v0
    1. OR Follow `Submitting from scratch` to push to dockerhub
2. follow these instructions: https://bitbucket.org/osrf/vrx-docker/src/default/ 

# Updating
0. get docker
1. Get the docker image: docker run tisbutascratch/usyd_vrx:v0

