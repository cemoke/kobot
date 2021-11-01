#!/bin/zsh
branch_dotfiles=master
branch_kobot=master
remote_dotfiles=origin
remote_kobot=origin

directory=~/kobot_ws/src/kobot
#Discard local changes and use latest from remote
echo "git reset --hard 2>&1"
git reset --hard  || { echo 'reset failed' ; exit 1; }
#Download changes from origin
echo "git pull $remote_dotfiles $branch_dotfiles 2>&1"
git pull $remote_dotfiles $branch_dotfiles 2>&1  || { echo 'fetch failed' ; exit 1; }