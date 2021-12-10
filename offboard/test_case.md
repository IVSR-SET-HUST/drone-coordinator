# OFFBOARD node - Test cases

## <span style="color:violet">Case 1: Hovering at z(m) - in simulation
```
roslaunch offboard offboard.launch simulation:=true
```
- <span style="color:cyan">In terminal
```
[ INFO] Please choose mode
- Choose 1 for hovering at Z(m)
- Choose 2 to fly with mission
(1/2):
```
<span style="color:orange">-> Input 1 to choose hovering mode

```
[ INFO] Mode 1: Hovering at Z(m)
        Input Z =
```
<span style="color:orange">-> Input z from keyboard

- <span style="color:cyan">Node will run itself

## <span style="color:violet">Case 2: Hovering at z(m) - in drone or hitl simulation
```
roslaunch offboard offboard.launch
```

- <span style="color:cyan">Similar in Case 1. After 'Input z from keyboard', terminal will show

```
[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC
```

- <span style="color:cyan">Use RC controller to set ARM and switch to OFFBOARD mode

## <span style="color:violet">Case 3: Mission mode, manual input local setpoints from keyboard, hovering at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true
```
- <span style="color:cyan">In terminal
```
[ INFO] Please choose mode
- Choose 1 for hovering at Z(m)
- Choose 2 to fly with mission
(1/2):
```
<span style="color:orange">-> Input 2 to choose mission mode

```
[ INFO] How do you want to input target/goal?
- Choose 1: Manual Input from keyboard
- Choose 2: Load prepared parameters from launch file
(1/2):
```
<span style="color:orange">-> Input 1 for manual input

```
[ INFO] Choose the setpoint type
- 3 for Local position
- 4 for Global position
(3/4):
```
<span style="color:orange">-> Input 3 for local setpoint

```
[ INFO] Input Local target position(s)
 Number of target(s):
```
<span style="color:orange">-> Input the number of setpoint in mission

```
 Target (1) postion (in meter):
   x(1): 0
   y(1): 0
   z(1): 5
 Target (2) postion (in meter):
   x(2): 
```
<span style="color:orange">-> Input position of each setpoint

```
 Position error check value (in meter): 
```
<span style="color:orange">-> Input offset value for check when drone reached setpoint (usually choose 0.1 or 0.2 m)

- <span style="color:cyan">Node will run itself

## <span style="color:violet">Case 4: Mission mode, manual input local setpoints from keyboard, landing for unpack cargo at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 3

## <span style="color:violet">Case 5: Mission mode, manual input local setpoints from keyboard, hovering at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch
```

- <span style="color:cyan">Similar in Case 3. After 'Input offset value for check when drone reached setpoint', terminal will show
```
[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC
```

- <span style="color:cyan">Use RC controller to set ARM and switch to OFFBOARD mode

## <span style="color:violet">Case 6: Mission mode, manual input local setpoints from keyboard, landing for unpack cargo at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 5

## <span style="color:violet">Case 7: Mission mode, manual input GPS setpoints from keyboard, hovering at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true
```
- <span style="color:cyan">In terminal
```
[ INFO] Please choose mode
- Choose 1 for hovering at Z(m)
- Choose 2 to fly with mission
(1/2):
```
<span style="color:orange">-> Input 2 to choose mission mode

```
[ INFO] How do you want to input target/goal?
- Choose 1: Manual Input from keyboard
- Choose 2: Load prepared parameters from launch file
(1/2):
```
<span style="color:orange">-> Input 1 for manual input

```
[ INFO] Choose the setpoint type
- 3 for Local position
- 4 for Global position
(3/4):
```
<span style="color:orange">-> Input 4 for GPS setpoint

```
[ INFO] Input Global goal position(s)
  Number of goal(s): 
```
<span style="color:orange">-> Input the number of setpoint in mission

```
  Goal (1) postion:
    Latitude (1) (in degree): 21.09384756
    Longitude (1) (in degree): 105.09384756
    Altitude (1) (in meter.above ground): 5
  Goal (2) postion:
    Latitude (2) (in degree):  
```
<span style="color:orange">-> Input position of each setpoint

```
 Position error check value (in meter): 
```
<span style="color:orange">-> Input offset value for check when drone reached setpoint (usually choose 0.1 or 0.2 m)

- <span style="color:cyan">Node will run itself

## <span style="color:violet">Case 8: Mission mode, manual input GPS setpoints from keyboard, landing for unpack cargo at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 7

## <span style="color:violet">Case 9: Mission mode, manual input GPS setpoints from keyboard, hovering at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch
```

- <span style="color:cyan">Similar in Case 7. After 'Input offset value for check when drone reached setpoint', terminal will show
```
[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC
```

- <span style="color:cyan">Use RC controller to set ARM and switch to OFFBOARD mode

## <span style="color:violet">Case 10: Mission mode, manual input GPS setpoints from keyboard, landing for unpack cargo at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 9

## <span style="color:violet">Case 11: Mission mode, load prepared setpoints from launch file, hovering at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true
```
- <span style="color:cyan">In terminal
```
[ INFO] Please choose mode
- Choose 1 for hovering at Z(m)
- Choose 2 to fly with mission
(1/2):
```
<span style="color:orange">-> Input 2 to choose mission mode

```
[ INFO] How do you want to input target/goal?
- Choose 1: Manual Input from keyboard
- Choose 2: Load prepared parameters from launch file
(1/2):
```
<span style="color:orange">-> Input 2 for load prepared setpoints

```
[ INFO] Choose the setpoint type
- 3 for Local position
- 4 for Global position
(3/4):
```
<span style="color:orange">-> Input 3 for loading local setpoints OR Input 4 for loading GPS setpoints

- <span style="color:cyan">Node will run itself

## <span style="color:violet">Case 12: Mission mode, load prepared setpoints from launch file, landing for unpack cargo at each setpoint - in simulation
```
roslaunch offboard offboard.launch simulation:=true delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 11

## <span style="color:violet">Case 13: Mission mode, load prepared setpoints from launch file, hovering at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch
```
- <span style="color:cyan">Similar in Case 11. After 'Load parameters', terminal will show
```
[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC
```

- <span style="color:cyan">Use RC controller to set ARM and switch to OFFBOARD mode

## <span style="color:violet">Case 14: Mission mode, load prepared setpoints from launch file, landing for unpack cargo at each setpoint - in drone or hitl simulation
```
roslaunch offboard offboard.launch delivery:=true
```
- <span style="color:cyan">Just change parameter `delivery` to `true`, other step is similar Case 13

## <span style="color:violet">Case 15: All caes of Mission mode (case 3 to case 14), add parameter 'return_home' 
```
roslaunch offboard offboard.launch [simulation:=true] [delivery:=true] return_home:=true
```
- <span style="color:cyan">Set parameter `return_home` to `true` for returning drone to start position. If set `false`, drone will landing at final setpoint