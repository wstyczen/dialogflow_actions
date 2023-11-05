# Action servers

## TurnToHuman

> When called, orients the robots to face the human.
>
> If possible only the head will move. If the goal is outside of its
> range of motion, the robot will rotate its torso beforehand.

### Running the server

```sh
roslaunch human_interactions turn_to_human_action_server.launch
```

### Testing the action

```sh
roslaunch human_interactions run_simulation # Launch the simulation for room 012
roslaunch human_interactions turn_to_human_action_server.launch # Launch the action server
roslaunch human_interactions turn_to_human_action_client.launch # Launch a simple client to trigger the action manually
```

## MoveToHuman

> This action moves the robot to the vicinity of the human, while maintaining safe distance.
>
> When the goal is reached the robot orients itself towards the human.

### Running the server

```sh
roslaunch human_interactions move_to_human_action_server.launch
```

### Testing the action

```sh
roslaunch human_interactions run_simulation # Launch the simulation for room 012
roslaunch human_interactions move_to_human_action_server.launch # Launch the action server
roslaunch human_interactions move_to_human_action_client.launch # Launch a simple client to trigger the action manually
```

## Launching all action servers

```sh
roslaunch human_interactions action_servers.launch # Launch all action servers
```

## Code **documentation**

> Sphynx documentation.

```sh
# From pkg root
sphinx-apidoc -o doc src # generate .rst files for python modules
make html # or 'rosdoc_lite .' -- generate html doc
```
