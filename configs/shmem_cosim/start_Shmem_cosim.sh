#!/bin/bash

#####################################################
# 'set -x' enables printing of all executed commmands
# 'set +x' disables ouput
######################################################
# set -x
set +x

#####################################################
# Print usage/help information
#####################################################
_echo_help() {
	echo "##############################################################"
	echo "USAGE: start_Shmem_cosim.sh [OPTIONS] -s <path_to_dpsim_scenario> \
			-v <path_to_villas_conf> -a <dpsim_scenario_arguments>"
	echo ""
	echo "Params:"
	echo -e "\t -s <path>\t\t Specify path to DPsim scenario "
	echo -e "\t -v <path>\t\t Specify path to VILLASnode config file"
	echo -e "\t -a <args>\t\t Specify arguments of the DPsim scenario (OPTIONAL)"
	echo ""
	echo "Options:"
	echo -e "\t -h | -? \t display this help information"
	echo -e "\t -p \t\t Use VILLASpipe instead of VILLASnode"
	echo "##############################################################"
}

#####################################################
# Parse command line options
#####################################################
DPSIM_ARG_STRING=""
while getopts "h?ps:a:v:" opt; do
    case "$opt" in
	h|\?)
		_echo_help
		exit 0
		;;
    p)
        PIPE=true
        ;;
	a)
		DPSIM_ARG_STRING=$OPTARG
		;;
	s)
		DPSIM_SCENARIO_PATH=$OPTARG
		;;
	v)
		VILLAS_CONFIG_PATH=$OPTARG
    esac
done

# Check if required file paths were specified
if [ -z "$DPSIM_SCENARIO_PATH" ]; then
	_echo_help
	echo -e "\033[0;31m Error: \033[0m No DPsim scenario path given!"
	exit 0
fi

if [ -z "$VILLAS_CONFIG_PATH" ]; then
	_echo_help
	echo -e "\033[0;31m Error: \033[0m No VILLASnode config given!"
	exit 0
fi

#####################################################
# Start villas-node/villas-pipe
#####################################################

if [ -z "$PIPE" ]; then
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node $VILLAS_CONFIG_PATH & VN=$!
else
	VILLAS_LOG_PREFIX="[Pipe] " \
	villas-pipe $VILLAS_CONFIG_PATH
fi

# Wait until node is successfully started
sleep 2

#####################################################
# Start simulation
#####################################################

TIME=$(date -d "+20 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Starting DPsim simulation at: $TIME ..."
CPS_LOG_PREFIX="[Sys ] " \
eval "$DPSIM_SCENARIO_PATH $DPSIM_ARG_STRING"


#####################################################
# Stop VILLASnode processes
#####################################################

if [ -z "$PIPE" ]; then
	kill $VN
else
	pkill villas-pipe
fi

#####################################################
# Catch signals
#####################################################

_stop() {
	echo "Caught SIGTSTP signal!"
	kill -TSTP ${CHILDS} 2>/dev/null
}

_cont() {
	echo "Caught SIGCONT signal!"
	kill -CONT ${CHILDS} 2>/dev/null
}

_term() {
	echo "Caught SIGTERM signal!"
	kill -TERM ${VN} ${CHILDS} 2>/dev/null
}

_kill() {
	echo "Caught SIGKILL signal!"
	kill -KILL ${VN} ${CHILDS} 2>/dev/null
}

trap _stop SIGTSTP
trap _cont SIGCONT
trap _term SIGTERM
trap _kill SIGKILL
trap _kill SIGINT
