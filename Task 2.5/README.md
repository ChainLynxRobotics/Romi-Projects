# Task 2.5: Creating a Subsystem

You'll build: The RomiDrivetrain template you used in Task 1.

**Time:** about 1 hour · **Need:** Nothing, but finishing Task 1 and Task 2 will help you build intuition.

> [!TIP]
> Stuck? Each step has a finished copy in [`checkpoints/`](checkpoints).

---

To start the process of creating a subsystem, first create a file for it. You should use the `Java Projects` view over the normal Explorer view since it groups by package and allows you to create Java files directly.

Create a new file by clicking on the plus on the `first.robot.mechanisms` package.

 <img src="./images/new_file.png" width="200"/>

Then select `Class`

 <img src="./images/new_class.png" width="400"/>

It will prompt you to name your new class. Call it `RomiDrivetrain`. Pressing enter should take you to a new blank class like below.

 <img src="./images/RomiDrivetrain_blank.png" width="800"/>

Next, we must implement `Mechanism`. All robot mechanisms (aka subsystems) must implement this interface. To implement simply add `implements Mechanism` after `RomiDrivetrain` like so:

```java
public class RomiDrivetrain implements Mechanism {

}
```

>[!TIP]
>`Mechanism` is located in the package `org.wpilib.command3`. We have to add the import to use it, but VSCode will suggest it as we type. Press [enter] to take the suggestion and VSCode will automatically add the needed import.

