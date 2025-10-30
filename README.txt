Creating ros2 workspace
- in the home directory in terminal run: git clone https://github.com/Cain-Poulton/IRS-Group-2-Project
- this should create a folder called IRS-Group-2-Project containing all relevant files

Docker and irs_simulation
- launch irs_simulation
- boot the docker either with compose or the docker executable on lab laptops

Booting articulation node
- start a terminal and cd into IRS-Group-2-Project: cd IRS-Group-2-Project
- create a colcon build: colcon build
- than source the build: source install/setup.bash
- launch with: ros2 launch tm12x_moveit_config hand_solo_moveit.launch.py

Booting navigation and main node
- open a new terminal and cd into IRS-Group-2-Project: cd IRS-Group-2-Project
- create a colcon build: colcon build
- than source the build: source install/setup.bash
- launch with: ros2 launch hand_solo_virtual_nav nav_launch.py

Booting PLC
- in a web browswer open: localhost:8080
- sign into the page with the username and password: openplc
- upload plc_system.st within the git clone IRS-Group-2-Project and then start it
- within the irs_simulation PLC UI turn on the PLC and conveyer (it should now be running)

The system should now be running, handsolo may be unable to navigate to the positions of small
and medium boxes, and on occasion may fail to grab large boxes. The dynamic avoidance of Kevin
has not been integrated.
