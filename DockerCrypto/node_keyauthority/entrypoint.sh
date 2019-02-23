#! /bin/sh

. /usr/src/node_keyauthority/devel/setup.sh
catkin_make
exec "$@"