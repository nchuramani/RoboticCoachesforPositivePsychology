echo "Executing on Pepper for user: " $1, $2
roslaunch dialogue_manager PepperDialogueManager.launch save_frames:=1 name:=$1 session:=$1 experimenter:=$2
