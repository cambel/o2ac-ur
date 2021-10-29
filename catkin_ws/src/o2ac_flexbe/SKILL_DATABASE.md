# Instructions
## Task Planner
To define a problem for the task planner in PDDL. Define the task based on the `o2ac_task_planning/pddl_converter/symbolic/domain_skill_db.pddl` domain. See examples `problem_assembly_subtask_a.pddl`, `problem_assembly_subtask_c2.pddl`, and `problem_assembly_subtask_f_g.pddl` in the same package.

To run the task planner:
1. Run `roscore` on a terminal
2. Launch the `fast-downward-server`: `rosrun downward fast-downward-server.py`
3. Execute the client script. E.g. using the `problem_assembly_subtask_a.pddl` file: `rosrun o2ac_task_planning_pddl_converter fast_downward_client.py domain_skill_db.pddl problem_assembly_subtask_a.pddl --search_output_file result_subtask_a`

The resulting PDDL trace will be store in `o2ac_task_planning/pddl_converter/symbolic/generated/result_subtask_a`

## Behavior Engine FlexBE
Load the PDDL trace as a FlexBE behavior.

1. Load FlexBE: `roslaunch flexbe_app flexbe_full.launch`
2. Go to the option `Load Task Skeleton` in the tool bar
3. Choose the PDDL trace
4. Save the behavior with the option `Save Behavior` in the tool bar

## Run behavior on the robotic system

1. Load the simulation environment such as: `roslaunch o2ac_moveit_config demo.launch`
1. Load the skill server: `rosrun o2ac_routines skill_server.py`
2. Go the FlexBE GUI
3. Go to the `Runtime Control` in the tool bar
4. `Start Execution` 