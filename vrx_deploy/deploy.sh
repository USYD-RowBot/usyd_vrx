CONTAINERNAME="tisbutascratch/usyd_vrx:v3"
docker rm updatesequence
docker run --name updatesequence --entrypoint "./update.sh" "$CONTAINERNAME"

echo "container update complete - comitting..."
sleep 1s

docker commit --change "ENTRYPOINT /ros_entrypoint.sh" updatesequence "$CONTAINERNAME"

docker rm updatesequence

echo "pushing in 5s..."

t=4
while [ "$t" -gt 0 ]    # wait 4 s
do
  echo "-- $t --"
   t=`expr $t - 1`
   sleep 1s
done

docker push "$CONTAINERNAME"

echo "done."