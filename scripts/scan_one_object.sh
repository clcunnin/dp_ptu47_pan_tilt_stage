# Topics to record
TOPICS="/narrow_stereo/left/image_raw /narrow_stereo/right/image_raw /narrow_stereo/left/camera_info /narrow_stereo/right/camera_info /narrow_stereo_textured/left/image_raw /narrow_stereo_textured/right/image_raw /narrow_stereo_textured/left/camera_info /narrow_stereo_textured/right/camera_info /dp_ptu47/pan_tilt_status /dp_ptu47/pan_tilt_status_stamped /tf"
# Trajectory to follow
TRAJECTORY="-150 0"
for i in -30 -20 -10 0 10 20
do
TRAJECTORY="$TRAJECTORY -150 $i 150 $i 150 `expr $i + 5` -150 `expr $i + 5`"
done
TRAJECTORY="$TRAJECTORY -150 0 0 0"

#echo $TOPICS
#echo $TRAJECTORY


if [ ! -z "$1" ]
then
    echo "Recording $1.bag"
    rosrecord -F $1 $TOPICS &
    rosrun dp_ptu47_pan_tilt_stage dp_ptu47_trigger 0 0
    sleep 1
    rosrun dp_ptu47_pan_tilt_stage dp_ptu47_trigger $TRAJECTORY
    ps | grep rosrecord | awk '{print $1}' | xargs kill -sigint
    echo "Done"
else
    echo "Usage:"
    echo "$0 object_name"
fi
