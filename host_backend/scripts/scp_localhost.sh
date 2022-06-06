#!/bin/bash
cur_path=$(dirname ${BASH_SOURCE[0]})
# aim to quickly create and copy the files on robots



#update clients
sshpass -p xtark ssh -tt xtark@$1 << eeooff 
cd /home/xtark/
rm -rf sim2real
mkdir sim2real
exit
eeooff

sshpass -p xtark scp -r ${cur_path}/../../../../* xtark@$1:/home/xtark/sim2real/

sshpass -p xtark ssh -tt xtark@$1 << eeooff 
cd /home/xtark/sim2real/sim2real/hosts/host_backend/
rm -rf devel/ build/
catkin_make
cd /home/xtark/sim2real/
pip install -e .
exit
eeooff
