#!/bin/bash
printf "************************************\n"
printf "*                                  *\n"
printf "*    Attention. Achtung. 1.3 Gb    *\n"
printf "*                                  *\n"
printf "************************************\n"
curl http://ec2-3-208-115-48.compute-1.amazonaws.com/ds.v001.tar.gz -O
tar -zxvf ds.v001.tar.gz
rm ds.v001.tar.gz
