#! /bin/bash

# Pull from git, build ground software, upload blastcmd, upload linklists

REPO_DIR="~/git/TIMflight"
GROUNDHOG_BUILD_DIR="$REPO_DIR/groundhog/build"
BLASTCMD_BUILD_DIR="$REPO_DIR/blastcmd"

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

if [ -z "$2" ]
then
  GIT_BRANCH="main"
else
  GIT_BRANCH="$2"
fi

GIT_PULL_CMD="cd $REPO_DIR; git pull; git checkout $GIT_BRANCH; git pull origin $GIT_BRANCH"

GROUNDHOG_BUILD_CMD="cd $GROUNDHOG_BUILD_DIR; cmake ../; make clean all; sudo make install"
GROUNDHOG_RESTART_CMD="sudo pkill groundhog"

BLASTCMD_BUILD_CMD="cd $BLASTCMD_BUILD_DIR; make; sudo make install"
BLASTCMD_RESTART_CMD="sudo pkill blastcmd"
BLASTCMD_UPLOAD="cd $BLASTCMD_BUILD_DIR; scp blastcmd fc1user@$fc1_ip:~/; scp blastcmd fc1user@$fc2_ip:~/"
BLASTCMD_INSTALL_FC="cd; install -m 755 -p blastcmd /usr/local/sbin/"

KILL_MCP="sudo kill -INT \$(pidof mcp) > /dev/null 2>&1"

LINKLIST_UPDATE_CMD="cd $REPO_DIR; ./upload_linklists.sh"
BLASTCMD_RESTART_FC="ssh -t $fc1_ip '$BLASTCMD_INSTALL_FC; $BLASTCMD_RESTART_CMD'; $KILL_MCP; ssh -t $fc2_ip '$BLASTCMD_INSTALL_FC; $BLASTCMD_RESTART_CMD'; $KILL_MCP"


ssh -t blast@$1 "$GIT_PULL_CMD; $LINKLIST_UPDATE_CMD; $GROUNDHOG_BUILD_CMD; $GROUNDHOG_RESTART_CMD; $BLASTCMD_BUILD_CMD; $BLASTCMD_RESTART_CMD; $BLASTCMD_UPLOAD; $BLASTCMD_RESTART_FC"
