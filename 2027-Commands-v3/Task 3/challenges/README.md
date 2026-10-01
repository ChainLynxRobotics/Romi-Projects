# Task 3 challenge answers

| Challenge | Answer | What you'll see |
|---|---|---|
| Figure 8 | [FigureEightAuto.java](FigureEightAuto.java) | A loop right, then a loop left, ending in the middle |
| Curve then Square | [CurveThenSquareAuto.java](CurveThenSquareAuto.java) | S-curve, then a clean PID square |

**Speed it up** (tested on the S-curve):

| `kMaxPathSpeedMetersPerSec` | Path time | Farthest off the path |
|---|---|---|
| 0.3 | 8.0 s | 6 cm |
| 0.45 | 6.3 s | 9 cm |
| 0.6 | 5.8 s | 10 cm |
| 0.8 | 5.3 s | 18 cm |
| 1.0 | 5.0 s | 35 cm |

Past **0.6** (`kMaxSpeedMetersPerSec`, the Romi's top speed) it can't keep up and cuts corners. The time barely improves because `kMaxPathAccelMetersPerSecSq` also limits how fast it can speed up.
