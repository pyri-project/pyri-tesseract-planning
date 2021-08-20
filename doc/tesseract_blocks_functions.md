# Tesseract Planner Blocks and Sandbox Functions

"Tesseract" section in the Blockly toolbox. Provides blocks and functions for using the Tesseract robot motion planner. These functions and blocks are provided by the `pyri-tesseract-planner` package. The environment containing the workspace and robot geometry must be loaded using the command line when the Tesseract planner service is started. Use `robot_set_active_robot()` to set the active robot.

## tesseract_get_robot_position

![](figures/blocks/tesseract_get_robot_position.png)

    tesseract_get_robot_position()

Get the position of the active virtual twin robot

## tesseract_plan_freespace_joint_target

![](figures/blocks/tesseract_plan_freespace_joint_target.png)

    tesseract_plan_freespace_joint_target(target_joint_position, speed_perc)

Plan collision-free freespace move with a joint target for
active robot

## tesseract_set_robot_position

![](figures/blocks/tesseract_set_robot_position.png)

    tesseract_set_robot_position(position)

Set the position of the active virtual twin robot

## tesseract_sync_robot

![](figures/blocks/tesseract_sync_robot.png)

    tesseract_sync_robot()

Sync virtual twin robot to real robot
