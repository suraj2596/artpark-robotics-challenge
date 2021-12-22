XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged -v /dev/video0:/dev/video0 \
    --privileged -v /dev/video1:/dev/video1 \
    --privileged -v /dev/video2:/dev/video2 \
    --privileged -v /dev/video3:/dev/video3 \
    --privileged -v /dev/video4:/dev/video4 \
    --privileged -v /dev/video5:/dev/video5 \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --device /dev/snd \
    --name cerberus3 \
    cerberus3
