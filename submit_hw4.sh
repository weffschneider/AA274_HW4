#!/bin/bash

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw4.zip"
echo "Creating ${sunetid}_hw4.zip"
zip -q "${sunetid}_hw4.zip" "submit_hw4.sh"
zip -qd "${sunetid}_hw4.zip" "submit_hw4.sh" # making an empty zip file

for fname in "Problem_1/astar.py" \
             "Problem_2/rrt.py" \
             "Problem_2/utils.py"
do
    if [ -f $fname ]; then
        zip "${sunetid}_hw4.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

got_screenshot=0
if [ -f "$sunetid.pdf" ]; then
    got_screenshot=1
    zip "${sunetid}_hw4.zip" "$sunetid.pdf"
fi
if [ -f "$sunetid.png" ]; then
    got_screenshot=1
    zip "${sunetid}_hw4.zip" "$sunetid.png"
fi
if [ -f "$sunetid.jpg" ]; then
    got_screenshot=1
    zip "${sunetid}_hw4.zip" "$sunetid.jpg"
fi
if [ ${got_screenshot} -eq "0" ]; then
    read -p "Problem 3 screenshot ($sunetid.pdf/png/jpg) not found. Skip it? [yn]: " yn
    case $yn in
        [Yy]* ) ;;
        * ) exit;;
    esac
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password if prompted."
ssh_result=$(ssh -o NumberOfPasswordPrompts=1 -o ControlMaster=auto -o ControlPersist=yes -o ControlPath=~/.ssh/%r@%h:%p $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW4 2>/dev/null)
ssh_success=$?
lastsub=$(echo ${ssh_result} | tr ' ' '\n' | grep -m 1 ${sunetid}_hw4_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
if [ $ssh_success -eq "255" ]; then
    tput setaf 1
    tput bold
    echo "
             / ██████╗ ███████╗████████╗██████╗ ██╗   ██╗██╗
  \    /\   |  ██╔══██╗██╔════╝╚══██╔══╝██╔══██╗╚██╗ ██╔╝██║
   )  ( ') <   ██████╔╝█████╗     ██║   ██████╔╝ ╚████╔╝ ██║
  (  /  )   |  ██╔══██╗██╔══╝     ██║   ██╔══██╗  ╚██╔╝  ╚═╝
   \(__)|   |  ██║  ██║███████╗   ██║   ██║  ██║   ██║   ██╗
             \ ╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝
An error occurred when connecting to the submission server. Please retry.
(One cause of this error message is entering your password incorrectly.)"
    exit
fi

subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password if prompted."
set -x
scp -o NumberOfPasswordPrompts=1 -o ControlMaster=auto -o ControlPersist=yes -o ControlPath=~/.ssh/%r@%h:%p "${sunetid}_hw4.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW4/${sunetid}_hw4_submission_$subnum.zip" 2>/dev/null
ssh_success=$?
{ set +x; } 2>/dev/null

if [ $ssh_success -eq "255" ]; then
    tput setaf 1
    tput bold
    echo "
             / ██████╗ ███████╗████████╗██████╗ ██╗   ██╗██╗
  \    /\   |  ██╔══██╗██╔════╝╚══██╔══╝██╔══██╗╚██╗ ██╔╝██║
   )  ( ') <   ██████╔╝█████╗     ██║   ██████╔╝ ╚████╔╝ ██║
  (  /  )   |  ██╔══██╗██╔══╝     ██║   ██╔══██╗  ╚██╔╝  ╚═╝
   \(__)|   |  ██║  ██║███████╗   ██║   ██║  ██║   ██║   ██╗
             \ ╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝
An error occurred when connecting to the submission server. Please retry.
(One cause of this error message is entering your password incorrectly.)"
    exit
else
    tput setaf 2
    tput bold
    echo "
             / ███████╗██╗   ██╗ ██████╗ ██████╗███████╗███████╗███████╗██╗
  \    /\   |  ██╔════╝██║   ██║██╔════╝██╔════╝██╔════╝██╔════╝██╔════╝██║
   )  ( ') <   ███████╗██║   ██║██║     ██║     █████╗  ███████╗███████╗██║
  (  /  )   |  ╚════██║██║   ██║██║     ██║     ██╔══╝  ╚════██║╚════██║╚═╝
   \(__)|   |  ███████║╚██████╔╝╚██████╗╚██████╗███████╗███████║███████║██╗
             \ ╚══════╝ ╚═════╝  ╚═════╝ ╚═════╝╚══════╝╚══════╝╚══════╝╚═╝"
fi