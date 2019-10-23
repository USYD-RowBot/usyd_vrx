CONTAINERNAME="tisbutascratch/usyd_vrx:v3"
docker rm updatesequence
docker run --name updatesequence --entrypoint /update.sh "$CONTAINERNAME"

t=10
while [ "$t" -gt 0 ]    # wait 10 s
do
  echo "--wait $t --"
   t=`expr $t - 1`
   sleep 1s
done

docker commit --change "ENTRYPOINT /ros_entrypoint.sh" updatesequence "$CONTAINERNAME"
t=4
while [ "$t" -gt 0 ]    # wait 4 s
do
  echo "--wait $t --"
   t=`expr $t - 1`
   sleep 1s
done
docker rm updatesequence
docker run "$CONTAINERNAME"