echo "Executing on Pepper for user: "  $1
roslaunch dialogue_manager PepperDialogueManager.launch save_frames:=1 NAME:=$1 session:=$1
