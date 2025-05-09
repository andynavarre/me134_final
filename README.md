# Multi-Robot Swarm Behavior with MQTT & AprilTag Vision

This repository contains the code and instructions to deploy a coordinated multi-robot system composed of three robots: `BASE`, `MIDDLE`, and `TOP`. The robots communicate via MQTT, align using AprilTags, and execute a sequence of collaborative tasks based on their assigned roles.

---

## Project Overview

- **Roles**: Robots autonomously determine their roles: `BASE`, `MIDDLE`, or `TOP`.
- **Communication**: MQTT handles all robot-to-robot messaging.
- **Vision**: AprilTag detection via HuskyLens enables physical alignment.
- **Behaviors**: Each role has a unique finite state machine to execute coordinated tasks.

---

## Prerequisites

- Python 3.x
- MQTT broker (e.g., [Mosquitto](https://mosquitto.org/))
- XRP Robot and Libraries
- HuskyLens camera set to `Tag Recognition Mode`
- Required Python libraries:
  - `XRPLib`
  - `HuskyLensLibrary`

##  Setup Instructions



### Set Up the MQTT Server

Run or connect to an MQTT broker (e.g., Mosquitto) on your local network.

Ensure all robots can publish and subscribe to the following topics:

- `swarm/broadcast`
- `swarm/role_assignment`
- `swarm/commands`
- `swarm/status`

---

##  Running the Robots

### Step-by-Step Procedure

1. **Power on and run the code on all three robots.**
   - Each robot will broadcast its ID and listen for role assignments.

2. **Wait for role assignment.**
   - One robot becomes the assigner.
   - It assigns: `BASE`, `MIDDLE`, and `TOP`.
   - Roles are repeatedly broadcast until all robots receive them.

3. **Manually stack the robots:**
   - **BASE** at the bottom
   - **MIDDLE** in the middle
   - **TOP** on top

4. **Press the button on the TOP robot.**
   - This begins the coordination sequence.
   - TOP starts moving; BASE and MIDDLE respond according to their roles.

5. **Robots execute their programmed behavior.**
   - BASE un-stacks and wall-follows
   - TOP line-follows and realigns
   - Re-stacking occurs after detecting final AprilTag

---

##  AprilTag Placement

Proper tag placement is critical to system performance.

| Location           | Purpose                                | Orientation                    |
|--------------------|----------------------------------------|--------------------------------|
| Wall (Start)       | BASE alignment before unstack          | Perpendicular to BASE camera   |
| Floor (Zone A)     | TOP alignment during line-following    | Flat, under TOP robot camera   |
| Wall (End)         | BASE final alignment before re-stack   | Facing BASE camera             |

- Use clear, high-contrast AprilTags.
- Ensure consistent lighting and unobstructed line of sight.
- Tags should be tested with HuskyLens beforehand.

---

##  Tuning Parameters

| Parameter                  | Value         | Notes                          |
|---------------------------|---------------|--------------------------------|
| Wall-following distance   | `10 cm`       | Adjusted for stable movement   |
| Wall lost threshold       | `50 cm`       | Detects when robot leaves path |
| PID line-following (TOP)  | `Kp=30, Ki=1` | Tuned via testing              |
| Turning calibration       | Custom        | Use timed `set_speed()` or `turn()` with tuning script |

---

##  Troubleshooting

-  **Robot stuck in `STATE_WAIT_ROLE`?**  
  Make sure it receives its role assignment on `swarm/role_assignment`.

-  **AprilTag not detected?**  
  Adjust lighting, camera angle, or tag size. Use HuskyLens tag viewer to verify.

-  **Unexpected movement or no movement?**  
  Check motor wiring, PID values, and make sure `drivetrain.set_speed()` is being called properly.
