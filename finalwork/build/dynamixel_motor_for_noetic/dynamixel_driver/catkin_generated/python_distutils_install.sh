#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/amadeus/forTurtle/install/lib/python3.8/site-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/amadeus/forTurtle/install/lib/python3.8/site-packages:/home/amadeus/forTurtle/build/lib/python3.8/site-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/amadeus/forTurtle/build" \
    "/usr/bin/python3" \
    "/home/amadeus/forTurtle/src/dynamixel_motor_for_noetic/dynamixel_driver/setup.py" \
     \
    build --build-base "/home/amadeus/forTurtle/build/dynamixel_motor_for_noetic/dynamixel_driver" \
    install \
    --root="${DESTDIR-/}" \
     --prefix="/home/amadeus/forTurtle/install" --install-scripts="/home/amadeus/forTurtle/install/bin"
