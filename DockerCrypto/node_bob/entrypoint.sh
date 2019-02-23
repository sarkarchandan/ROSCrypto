#! /bin/sh

. /usr/src/node_bob/devel/setup.sh
catkin_make
exec "$@"