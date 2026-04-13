# Controller Bindings

This document outlines all controller bindings defined in `robot_container.py` (`_setup_controller_bindings`).

---

## Controllers

| Controller | Port | Role |
|------------|------|------|
| **Driver Controller** | 0 | Driving and intake |
| **Function Controller** | 1 | Superstructure (aim, launch), launcher state, override, climber |

---

## Driver Controller (Port 0)

### Default Command
- **Field-centric drive** (always active unless overridden)
  - Left stick Y → Velocity X (forward/backward)
  - Left stick X → Velocity Y (strafe left/right)
  - Right stick X → Rotational rate (turn)

### Buttons

| Button | Action | Trigger Type | Description |
|--------|--------|--------------|-------------|
| **Left Bumper** | Robot-centric drive | `whileTrue` | Overrides field-centric; drives relative to robot heading |
| **Right Bumper** | Intake | `whileTrue` / `onFalse` | While held: INTAKE. On release: STOP |
| **Right Trigger** | Intake OUT (OUTPUT) | `whileTrue` / `onFalse` | While axis > 0.75: OUTPUT. On release: STOP |
| **A** | Brake | `whileTrue` | Swerve brake; wheels lock in current orientation |
| **X** | Point wheels | `whileTrue` | Points wheels in direction of left stick (Y, X) |
| **Start** | Seed field-centric | `onTrue` | Resets robot heading for field-centric orientation |

---

## Function Controller (Port 1)

### Buttons (superstructure — requires **turret and hood** on the robot)

| Button | Action | Trigger Type | Description |
|--------|--------|--------------|-------------|
| **Y** | Superstructure → AIMHUB | `onTrue` | Aim at hub (turret, hood, launcher setpoints via superstructure) |
| **X** | Superstructure → AIMDEPOT | `onTrue` | Aim at depot pass target |
| **B** | Superstructure → AIMOUTPOST | `onTrue` | Aim at outpost pass target |
| **A** | Superstructure → DEFAULT | `onTrue` | Feeder STOP, launcher IDLE, hood STOW; intake/turret are not changed by this goal (see `_goal_to_states` in `superstructure.py`) |
| **Start** | Toggle checker override | `onTrue` | Toggles `Superstructure` feeder-alignment check override |

### Triggers (analog) — requires **launcher** on the robot

| Trigger | Threshold | Action | Trigger Type | Description |
|---------|-----------|--------|--------------|-------------|
| **Right Trigger** | > 0.75 | LAUNCH / STOPLAUNCH | `whileTrue` / `onFalse` | While held: `Superstructure.Goal.LAUNCH`. On release: `Superstructure.Goal.STOPLAUNCH` |
| **Left Trigger** | > 0.75 | Launcher SCORE / IDLE | `whileTrue` / `onFalse` | While held: launcher SCORE. On release: launcher IDLE |

### POV — requires **climber** on the robot

| POV | Action | Trigger Type | Description |
|-----|--------|--------------|-------------|
| **Up** | Climber EXTEND | `onTrue` | Climber extends |
| **Down** | Climber STOW | `onTrue` | Climber stows |

---

## Notes

- **Intake** (driver right bumper and right trigger OUTPUT): registered only when `intake` is not `None`.
- **Launcher triggers**: registered only when `launcher` is not `None`.
- **Y / X / B / A / start** (superstructure): registered only when **both** `turret` and `hood` are not `None`.
- **Climber**: registered only when `climber` is not `None`.

If a subsystem is missing, its bindings are skipped and a console message may be printed at startup.
