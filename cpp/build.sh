#!/bin/bash

image="pc_plotter"
tag="cpp"
exec_pwd=$(cd $(dirname $0); pwd)

docker build $exec_pwd \
    -t $image:$tag