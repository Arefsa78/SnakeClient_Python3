#!/bin/sh

echo "******************************************************************"
echo " SnakeClient_Cpp"
echo " Created by Nader Zare"
echo " Copyright 2019-.  Nader Zare"
echo " All rights reserved."
echo "******************************************************************"

player="client.py"
type="best" #greedy, random, hand, best, your
host="127.0.0.1"
port=20002
teamname="SnakeBase"

if [ $# -eq 2 ]
then
	type=$1
	teamname=$2
fi

if [ $# -eq 2 ]
then
	teamname=$2
fi

opt="-s ${host} -p ${port} -n ${teamname} -c ${type}"

python3 $player ${opt}
