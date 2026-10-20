# Romi Troubleshooting Guide

## Setup

| Problem | Fix |
|---|---|
| Build error about **Java version** / `Math.clamp` / `org.wpilib` not found | You're in the wrong VS Code. Open **WPILib VS Code 2027** ([Task 0](Task%200/Instructions.md)). |
| "Vendor dependency" errors | Use the vendordeps that came with the Template. Old 2026 vendordeps don't work with 2027. |
| Can't find the Romi's WiFi | Romi on? Batteries charged? Right Romi (name on the bottom)? Someone else already connected? |
| Romi won't connect | Update its firmware: [Romi imaging guide](https://docs.wpilib.org/en/stable/docs/romi-robot/imaging-romi.html) |

## Running code

| Problem | Fix |
|---|---|
| How do I run it? | <img src="Task%201/wpilib%20logo.png" alt="WPILib logo" width="16"/> (top right) → **Simulate Robot Code** → **Sim GUI** |
| My OpMode isn't in the list | It must be in `frc/robot/opmodes/`, be `public`, have `@Teleop` or `@Autonomous`, and have a constructor that takes `Robot`. |
| Pressed Enable, nothing moves | Did you pick an OpMode? Teleop needs **Drive**. For a real Romi, set `useRealRomi = true` in `build.gradle`. |
| Pretend Romi doesn't move | The robot must be **enabled**. For sim-only, `useRealRomi = false` in `build.gradle`. |
| Everything freezes | A `while` loop inside a command is missing `coroutine.yield();` |
| `Command` won't compile: "named" | Every command needs `.named("Some Name")` at the end. |

## Sensors & motors (real Romi)

| Problem | Fix |
|---|---|
| Drives backward | Flip the `-` signs on the joystick lines in `DriveTeleop`. |
| Heading goes **down** when turning left | Check the `-` in `getHeadingDegrees()` (Task 1 Step 4a). |
| PID never finishes | Real motors need more push. Raise `kDriveP` / `kTurnP`. |

## Tips
- Watch numbers: open the **NetworkTables** view, or graph them with **Plot → New Plot Window**.
- `System.out.println("hi");` prints to the VS Code terminal.

## More help
- [WPILib Romi docs](https://docs.wpilib.org/en/stable/docs/romi-robot/index.html)
- [Commands v3 design doc](https://github.com/wpilibsuite/allwpilib/blob/main/design-docs/commands-v3.md)
- Ask an experienced team member!
