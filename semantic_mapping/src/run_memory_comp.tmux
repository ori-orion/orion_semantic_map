!/bin/bash

# Adapted from orion_hsr_bringup.

SESSION=memory

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
# DEVELOPMENT_WS_PC=/home/orion/wrs_ws/devel/setup.bash 	# old workspace directory
DEVELOPMENT_WS_PC=/home/$USER/orion_ws/devel/setup.bash		# updated and standardised orion_ws directory
DEVELOPMENT_WS_PC_2=/home/orion/robocup_ws/devel/setup.bash
DEVELOPMENT_WS_ROBOT=/home/administrator/orion_ws/devel/setup.bash
_SRC_ENV_PC="tmux -u send-keys source Space $DEVELOPMENT_WS_PC C-m"
_SRC_ENV_PC_2="tmux -u send-keys source Space $DEVELOPMENT_WS_PC_2 C-m"
_SRC_ENV_PC_3="tmux -u send-keys source Space activate Space tf2 C-m"
_SRC_ENV_ROBOT="tmux -u send-keys source Space $DEVELOPMENT_WS_ROBOT C-m"

CLEAR_PANE='tmux -u send-keys "clear" Enter'

# This is for setting up the Ethernet connection and needs to be run per terminal.
PREFIX = init_hsr_ethernet

tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'mongod'
tmux new-window -t $SESSION:1 -n 'semantic_mapping'
tmux new-window -t $SESSION:2 -n 'detections_to_observations'

tmux select-window -t $SESSION:0
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV_ROBOT` && `$CLEAR_PANE`
tmux -u send-keys "mongod --dbpath /home/$USER/orion_ws/db" C-m

tmux select-window -t $SESSION:1
<<<<<<< HEAD
$PREFIX
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV_ROBOT` && `$CLEAR_PANE`
tmux -u send-keys "rosrun semantic_mapping main.py" C-m

=======
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV_ROBOT` && `$CLEAR_PANE`
tmux -u send-keys "rosrun semantic_mapping main.py" C-m

tmux select-window -t $SESSION:2
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV_ROBOT` && `$CLEAR_PANE`
tmux -u send-keys "rosrun semantic_mapping detections_to_observations.py" C-m

>>>>>>> matthew_munks
# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
