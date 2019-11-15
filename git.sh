#!/bin/bash
echo -e "\E[1;32mupdate from github ...\E[0m"
git fetch origin master
git merge origin/master
git rebase origin/master

echo -e "\E[1;32mupload to github ...\E[0m"

git push origin master


