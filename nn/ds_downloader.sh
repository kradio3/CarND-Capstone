#!/bin/bash
printf "************************************\n"
printf "*                                  *\n"
printf "*    Attention. Achtung. 1.3 Gb    *\n"
printf "*                                  *\n"
printf "************************************\n"
curl http://ec2-3-208-115-48.compute-1.amazonaws.com/sim_camera_ds.v002.tar.gz -O
curl http://ec2-3-208-115-48.compute-1.amazonaws.com/real_camera_ds.v001.tar.gz -O
tar -zxvf sim_camera_ds.v002.tar.gz
tar -zxvf real_camera_ds.v001.tar.gz
rm real_camera_ds.v001.tar.gz
rm sim_camera_ds.v002.tar.gz
