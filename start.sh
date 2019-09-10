#!/bin/sh

echo "******************************************************************"
echo " SnakeClient_Cpp"
echo " Created by Nader Zare"
echo " Copyright 2019-.  Nader Zare"
echo " All rights reserved."
echo "******************************************************************"

player="client.py"
type="your" #greedy, random, hand, best, your
host="127.0.0.1"
port=20002
teamname="CyrusSnake"

if [ $# -eq 2 ]
then
	type=$1
	teamname=$2
fi

if [ $# -eq 1 ]
then
    echo "${2}"
	teamname=$1
fi

opt="-s ${host} -p ${port} -n ${teamname} -c ${type}"

python3 $player ${opt}
