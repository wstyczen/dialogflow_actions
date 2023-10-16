# Dialogflow action servers

## TurnToHuman

> When called, orients the robots to face the human.
>
> If possible only the head will move, but if the goal is outside of its
> range of motion, the robot will rotate its torso instead.

### Running the server

```sh
roslaunch dialogflow_actions turn_to_human_action_server.launch
```

### Testing the action

```sh
roslaunch dialogflow_actions run_simulation # Launch the simulation for room 012
roslaunch dialogflow_actions turn_to_human_action_server.launch # Launch the action server
roslaunch dialogflow_actions turn_to_human_action_client.launch # Launch a simple client to trigger the action manually
```

## MoveToHuman

> This action moves the robot to the vicinity of the human, while maintaining safe distance.
>
> When the goal is reached the robot orients itself towards the human.

### Running the server

```sh
roslaunch dialogflow_actions move_to_human_action_server.launch
```

### Testing the action

```sh
roslaunch dialogflow_actions run_simulation # Launch the simulation for room 012
roslaunch dialogflow_actions move_to_human_action_server.launch # Launch the action server
roslaunch dialogflow_actions move_to_human_action_client.launch # Launch a simple client to trigger the action manually
```

## Launching all action servers

```sh
roslaunch dialogflow_actions action_servers.launch # Launch all action servers
```
