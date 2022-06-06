#!/bin/bash
cur_path=$(dirname ${BASH_SOURCE[0]})
sim2real_path=${cur_path}/../../../..
# aim to quickly update some py scripts on robots
rsync -av --exclude-from="${sim2real_path}/.gitignore" ${sim2real_path}/ xtark@$1:/home/xtark/sim2real/