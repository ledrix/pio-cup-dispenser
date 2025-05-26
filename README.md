# NEUROMEKA

Modbus TCP Communication

## Control Var

| Motor | Variable |
|-------|----------|
| 1     | 901      |
| 2     | 902      |
| 3     | 903      |
| 4     | 904      |

#### Values

| Value | Function     |
|-------|--------------|
| 0     | Idle         |
| 1     | Dispense     |
| 2     | **Reserved** |
| 3     | Close        |

## State Var

| Motor | Variable |
|-------|----------|
| 1     | 905      |
| 2     | 906      |
| 3     | 907      |
| 4     | 908      |

#### Values

| Value | Function   |
|-------|------------|
| 0     | Idle       |
| 1     | Dispensing |
| 2     | Dispensed  |
| 3     | Closing    |


## Flow

| Step | Control | State | Description              |
|------|---------|-------|--------------------------|
| 1    | 0       | 0     | Idle                     |
| 2    | 1       | 0     | Command: Cup Release     |
| 3    | 1       | 1     | Dispensing               |
| 4    | 1       | 2     | Cup Dispensed            |
| 5    | 1       | 2     | Robot: Removes Cup       |
| 6    | 3       | 2     | Command: Close Dispenser |
| 7    | 3       | 3     | Closing Dispenser        |
| 8    | 3       | 0     | Dispenser Closed         |

**OBS:** Before commanding a cup dispensing, the robot must reset the control and state variables to zero

# KUKA

KukaVar Proxy Communication

## Variables

| Motor | Variable         |
|-------|------------------|
| 1     | CUP_DISPENSER[1] |
| 2     | CUP_DISPENSER[2] |
| 3     | CUP_DISPENSER[3] |
| 4     | CUP_DISPENSER[4] |

#### Values

| Value | Function   |
|-------|------------|
| 0     | Idle       |
| 1     | Dispense   |
| 2     | Dispensed  |
| 3     | Retract    |


## Flow

| Step | Robot   | PCB   | Description              |
|------|---------|-------|--------------------------|
| 1    | 0       | 0     | Idle                     |
| 2    | 1*      | 0     | Command: Cup Release     |
| 3    | 1       | 1*    | Dispensing               |
| 4    | 1       | 2*    | Cup Dispensed            |
| 5    | 1       | 2     | Robot: Removes Cup       |
| 6    | 3*      | 2     | Command: Close Dispenser |
| 7    | 3       | 3*    | Closing Dispenser        |
| 8    | 3       | 0*    | Dispenser Closed         |

**\*** Responsible for changing the variable value