echo "Executing simulation mode for user: "  $1, $2
roslaunch dialogue_manager ComputerDialogueManager.launch save_frames:=1 name:=$1 experimenter:=$2
