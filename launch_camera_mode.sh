echo "Executing simulation mode for user: "  $1
roslaunch dialogue_manager ComputerDialogueManager.launch save_frames:=1 NAME:=$1 session:=$1
