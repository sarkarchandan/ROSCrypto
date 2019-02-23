#! /bin/sh

. /usr/src/node_alice/devel/setup.sh
catkin_make
exec "$@"