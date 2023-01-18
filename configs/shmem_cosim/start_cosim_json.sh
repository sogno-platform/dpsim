#!/bin/bash
###########################################################################################
# Configuration parameters
###########################################################################################
_parseConfig() {
    NUMBER_OF_LOOPS=$(jq -r '.number_of_loops' $CONFIG_FILE)     	        # Number of loops that should be executed

    VILLAS_CONFIG_PATH=$(jq -r '.villas.config_path' $CONFIG_FILE)          # VILLASnode config

    DPSIM_USE_DOCKER=$(jq -r '.dpsim | has("docker")' $CONFIG_FILE)         # Indicate if DPsim is executed in Docker
    DPSIM_PATH=$(jq -r '.dpsim.path' $CONFIG_FILE)                          # Path to dpsim main dir
    DPSIM_SCENARIO_DIR=$(jq -r '.dpsim.scenario.directory' $CONFIG_FILE)    # Path to the scenario file
    DPSIM_SCENARIO_NAME=$(jq -r '.dpsim.scenario.name' $CONFIG_FILE)        # Name of the scenario file
    DPSIM_SCENARIO_ARGS=$(jq -r '.dpsim.scenario.args' $CONFIG_FILE)        # Arguments of the scenario
    DPSIM_LOG_DIR=$(jq -r '.logging.dpsim.directory' $CONFIG_FILE)          # Path to DPsim log directory
    DPSIM_LOG_FILES=$(jq -r '.logging.dpsim.files' $CONFIG_FILE)            # Space sep. list of DPsim log files

    DOCKER_IMAGE_NAME=$(jq -r '.dpsim.docker.image.name' $CONFIG_FILE)      # Name of dpsim_dev docker image
    DOCKER_IMAGE_TAG=$(jq -r '.dpsim.docker.image.tag' $CONFIG_FILE)            # Tag of dpsim_dev docker image
    DOCKER_CONTAINER_NAME=$(jq -r '.dpsim.docker.container_name' $CONFIG_FILE)  # Name of dpsim_dev docker container

    REMOTE_SIM_PATH=$(jq -r '.remote_sim.path' $CONFIG_FILE)                    # Path to remote sim directory
    REMOTE_SIM_LOG_DIR=$(jq -r '.logging.remote_sim.directory' $CONFIG_FILE)    # Path to logs of remote sim
    REMOTE_SIM_LOG_FILES=$(jq -r '.logging.remote_sim.files' $CONFIG_FILE)      # List of logged remote files

    LOGGING=$(jq -r 'has("logging")' $CONFIG_FILE)                              # "enable"/"disable" logging of results
    LOG_FILES_PATH=$(jq -r '.logging.log_file_destination' $CONFIG_FILE)        # Directory where results should be stored

    ###########################################################################################
    # Define simulator executables...
    ###########################################################################################
    REMOTE_SIM_EXEC=$(jq -r '.remote_sim.exec_command' $CONFIG_FILE)

    if [ "$DPSIM_USE_DOCKER" = "true" ]; then
        DPSIM_EXEC="docker exec -w /dpsim $DOCKER_CONTAINER_NAME \
            ./configs/shmem_cosim/start_Shmem_cosim.sh \
            -s $DPSIM_SCENARIO_DIR/$DPSIM_SCENARIO_NAME \
            -v $VILLAS_CONFIG_PATH"

        # append arguments if specified
        if [ "$DPSIM_SCENARIO_ARGS" != "null" ]; then
            DPSIM_EXEC="$DPSIM_EXEC -a $DPSIM_SCENARIO_ARGS"
        fi
    else
        DPSIM_EXEC="./configs/shmem_cosim/start_Shmem_cosim.sh \
            -s $DPSIM_SCENARIO_DIR/$DPSIM_SCENARIO_NAME \
            -v $VILLAS_CONFIG_PATH"

        # append arguments if specified
        if [ "$DPSIM_SCENARIO_ARGS" != "null" ]; then
            DPSIM_EXEC="$DPSIM_EXEC -a $DPSIM_SCENARIO_ARGS"
        fi
    fi
}



#####################################################
# Print usage/help information
#####################################################
_echoHelp() {
	echo "##############################################################"
	echo "USAGE: start_cosim.sh -f <config> [OPTIONS]"
	echo ""
	echo "Options:"
    echo -e "\t -h | -? \t display this help information"
    echo -e "\t -l <loops>\t specifiy number of loops (default=1)"
    echo -e "\t -c \t\t only print config"
	echo -e "\t -v \t\t verbose - print all executed commands to terminal"
	echo "##############################################################"
}

###########################################################################################
# Kill and remove docker container specified by $DOCKER_CONTAINER_NAME
###########################################################################################
_removeContainer(){
    docker kill $DOCKER_CONTAINER_NAME
    docker rm $DOCKER_CONTAINER_NAME
    echo "Killed and removed docker container!"
}

###########################################################################################
# Create and run docker container for DPsim
###########################################################################################
_runContainer(){
    echo "Starting fresh instance of docker container..."
    docker run -t --name $DOCKER_CONTAINER_NAME -d --network=host -v $(pwd):/dpsim \
        --privileged $DOCKER_IMAGE_NAME:$DOCKER_IMAGE_TAG bash
}

###########################################################################################
# Print used configuration
###########################################################################################
_printConfig(){
    echo "###############################################################"
    echo "###############################################################"
    echo "Co-simulation Configuration:"
    echo -e "\tNUMBER_OF_LOOPS:     \t$NUMBER_OF_LOOPS"
    echo -e ""
    echo -e "\tVILLAS_CONFIG_PATH:  \t$VILLAS_CONFIG_PATH"
    echo -e ""
    echo -e "\tLOGGING: \t\t$LOGGING"
    if [ "$LOGGING" = "true" ]; then
        echo -e "\tLOG_FILES_PATH: \t$LOG_FILES_PATH"
    fi
    echo -e ""
    echo -e "\tDPSIM_USE_DOCKER:    \t$DPSIM_USE_DOCKER"
    echo -e "\tDPSIM_PATH:          \t$DPSIM_PATH"
    echo -e "\tDPSIM_SCENARIO_DIR: \t$DPSIM_SCENARIO_DIR"
    echo -e "\tDPSIM_SCENARIO_NAME: \t$DPSIM_SCENARIO_NAME"
    echo -e "\tDPSIM_SCENARIO_ARGS: \t$DPSIM_SCENARIO_ARGS"
    if [ "$LOGGING" = "true" ]; then
        echo -e "\tDPSIM_LOG_DIR:       \t$DPSIM_LOG_DIR"
        echo -e "\tDPSIM_LOG_FILES:     \t$DPSIM_LOG_FILES"
    fi
    echo -e ""
    if [ "$DPSIM_USE_DOCKER" = "true" ]; then
        echo -e "\tDOCKER_IMAGE_NAME:       \t$DOCKER_IMAGE_NAME"
        echo -e "\tDOCKER_IMAGE_TAG:        \t$DOCKER_IMAGE_TAG"
        echo -e "\tDOCKER_CONTAINER_NAME:   \t$DOCKER_CONTAINER_NAME"
    fi
    echo -e ""
    echo -e "\tREMOTE_SIM_PATH:         \t$REMOTE_SIM_PATH"
    if [ "$LOGGING" = "true" ]; then
        echo -e "\tREMOTE_SIM_LOG_DIR:      \t$REMOTE_SIM_LOG_DIR"
        echo -e "\tREMOTE_SIM_LOG_FILES:    \t$REMOTE_SIM_LOG_FILES"
    fi
    echo "###############################################################"
    echo "###############################################################"
}

###########################################################################################
# Parse command line options
###########################################################################################
CONFIG_FILE=null

while getopts "h?f:l:cv" opt; do
    case "$opt" in
    h|\?)
        _echoHelp
        exit 0
        ;;
    f)
        CONFIG_FILE=$OPTARG
        _parseConfig
        ;;
    l)
        NUMBER_OF_LOOPS=$OPTARG
        ;;
    c)
        _printConfig
        exit 0
        ;;
    v)
        set -x
    esac
done

if [ "$CONFIG_FILE" = "null" ]; then
    echo ""
    echo -e "\033[0;31m Error: \033[0m No config file given! Missing: -f <config>"
    echo -e " USAGE: start_cosim.sh \033[0;33m -f <config> \033[0m[OPTIONS]"
    echo ""
    exit -1
fi

###########################################################################################
# Clean up remaining docker containers if previous run crashed or container name for some
# reason exists. Start a fresh instance afterwards!
###########################################################################################
if [ "$DPSIM_USE_DOCKER" = "true" ]; then
    _removeContainer
    _runContainer
    sleep 2	 # Make sure the container is successfully created
fi

###########################################################################################
# Prepare logging
#
# - Create folder for results of current session
# - remove whitespaces that are somehow introduced by 'date'...
###########################################################################################
if [ "$LOGGING" = "true" ]; then
    LOG_FILES_PATH_SESSION="$(echo -e $LOG_FILES_PATH/$(date +'%d-%m-%y_%r') \
        | tr -d '[:space:]')"
    mkdir -v -p "$LOG_FILES_PATH_SESSION"
fi
# touch "$LOG_FILES_PATH_SESSION/executionTimes.csv"

###########################################################################################
# Start cosimulation...
###########################################################################################
_printConfig
for ((i=1; i<=$NUMBER_OF_LOOPS; i++)); do
    # Execute simulations
    $REMOTE_SIM_EXEC&
    $DPSIM_EXEC

    # Wait for everything to finish...
    sleep 5

    # Save results of both simulations
    # TODO: Does not work like that if remote sim on another host! (use scp?)
    if [ "$LOGGING" = "true" ]; then
        if (( $NUMBER_OF_LOOPS > 1 )); then
            mkdir $LOG_FILES_PATH_SESSION/${i}
            LOG_FILES_PATH_RUN=$LOG_FILES_PATH_SESSION/${i}
        else
            LOG_FILES_PATH_RUN=$LOG_FILES_PATH_SESSION
        fi

        for FILE in ${DPSIM_LOG_FILES}; do
            cp $DPSIM_PATH/$DPSIM_LOG_DIR/$DPSIM_SCENARIO_NAME/$FILE $LOG_FILES_PATH_RUN
        done;

        for FILE in ${REMOTE_SIM_LOG_FILES}; do
            cp $REMOTE_SIM_PATH/$REMOTE_SIM_LOG_DIR/$FILE $LOG_FILES_PATH_RUN
        done
    fi

    # Wait for everything to finish...
    sleep 5

done

###########################################################################################
# Clean up docker container
###########################################################################################
if [ "$DPSIM_USE_DOCKER" = "true" ]; then
    _removeContainer
fi

###########################################################################################
# Catch SIGINT and cleanup
###########################################################################################
_kill(){
	echo "Caught SIGINT signal!"
	if [ "$DPSIM_USE_DOCKER" = "true" ]; then
        _removeContainer
    fi
    kill -KILL $$
}

trap _kill 2 INT SIGINT
