#!/bin/bash
# run_husky.sh
# Usage:
#   ./run_husky.sh build                # build the Docker image
#   ./run_husky.sh run                  # run a new Docker container
#   ./run_husky.sh exec                  # open a bash in the existing container

IMAGE_NAME="ros2-jazzy-ubuntu24"
CONTAINER_NAME="jazzy_container"

# Check argument
if [ $# -lt 1 ]; then
    echo "Usage: $0 [build|run|exec]"
    exit 1
fi

COMMAND=$1

case "$COMMAND" in
    build)
        echo "Building Docker image: $IMAGE_NAME"
        docker build -t $IMAGE_NAME .
        exit 0
        ;;
    run)
        echo "Running Docker container: $IMAGE_NAME"
        
        # Allow container to access X11
        xhost +local:root
        
        # Run the container (detached mode, so we can exec later)
        docker run -it \
            --rm \
            --name $CONTAINER_NAME \
            --gpus all \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --env="NVIDIA_DRIVER_CAPABILITIES=all" \
            --env="NVIDIA_VISIBLE_DEVICES=all" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume=$(pwd)/clearpath:/root/clearpath \
            --volume=$(pwd)/husky_ws:/root/husky_ws \
            --volume=$(pwd)/visualize.py:/root/visualize.py \
            --volume=$(pwd)/experiment_data:/root/experiment_data \
            --volume=$(pwd)/visualized_data:/root/visualized_data \
            --workdir="/root/husky_ws" \
            --net=host \
            $IMAGE_NAME
        
        echo "Container '$CONTAINER_NAME' started. Use './run_husky.sh exec' to open another terminal shell in it."
        exit 0

        ;;
    bash)
        # Check if container is running
        RUNNING=$(docker ps -q -f name=$CONTAINER_NAME)
        if [ -z "$RUNNING" ]; then
            echo "Error: Container '$CONTAINER_NAME' is not running. Use './run_husky.sh run' first."
            exit 1
        fi

        echo "Opening a new bash shell in container '$CONTAINER_NAME'"
        docker exec -it $CONTAINER_NAME bash
        exit 0
        ;;
    *)
        echo "Invalid argument: $COMMAND"
        echo "Usage: $0 [build|run|bash]"
        exit 1
        ;;
esac

      --volume=/tmp/.X11-unix/:/tmp/.X11-unix/\
      --volume=$(pwd)/$IMAGE:/home/$IMAGE \
      --workdir="/home/$IMAGE" \
      --env HOME="/home/$IMAGE" \