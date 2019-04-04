# hovercraft-ros
Development of ROS code for a small hovercraft

Keep in mind that at this stage everything is very experimental and far from being easily usable.


# Getting started

Activate the Python 2.7 virtual environment:
`source ~/virtualenvs/venv_hovercraft_2.7/bin/activate`

Navigate to the ROS workspace `hovercraft_ws` and source the setup file:
`source devel/setup.bash`

Launch the `vrpn_client_ros`:
`roslaunch vrpn_client_ros sample.launch server:=optitrack`

Launch the script to send PRBS commands to the hovercraft:
`rosrun ground test_communication.py`

To log the data sent to the hovercraft and the data from the tracking software:
`rosrun ground logger.py`
