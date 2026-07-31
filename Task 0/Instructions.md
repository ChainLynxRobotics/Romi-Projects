# Task 0: Configuring Computer

## Overview

It is recommended to bring your own laptop as school laptops have limitations. We have 2 drive laptops that can sometimes be used for coding, but usually they are being used to test the robot.

Installing WPILib VScode is very necessary. Normal VScode won't work, as it is missing critical tools required to run and simulate the robot.

## Downloading WPILib

Go to https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html#downloading,  and download the installer.

<img src="./Download.png" alt="Download button" width="901"/>

Then follow the install instructions below the download button.

## Downloading & configuring Git
### Windows
If you use a package manager like [winget](https://learn.microsoft.com/en-us/windows/package-manager/) or [chocolatey](https://chocolatey.org/) use those, otherwise go to https://git-scm.com/install/windows and download and run the standalone installer.

### MacOS
If you already have homebrew then use `brew install git`, otherwise use `xcode-select --install` to install the xcode command line tools which comes with git.

### Linux
Its probably already installed, you can check with `which git` if not use `<your favorate package manager invocation to install packages> git`.

In order to use git you will need set some config, all should be set with `git config --global <the setting>`
- Your name. `user.name=<your name>` You should generally set this to the same thing as your github username
- An email. Go to https://github.com/settings/emails at the bottom turn on the `Keep my email addresses private` setting, then find the email address in that box below the setting that looks like `number+username@users.noreply.github.com` then set that in git `user.email=<email you just found>`

There a few more settings you should set
- `rerere.enabled=true`
- `push.autosetupremote=true`
- `pull.rebase=true`
- `pull.merge=false`
- `rebase.autostash=true`

## Connecting to the Romis
To connect to the romis, disconnect from your current wi-fi and find the romi's network. If you can't find it, here are some common causes:

- Romi is off
- Romi is out of battery
- You are attempting to connect to the wrong romi
- You have wi-fi disabled
- Your computer just doesn't work
- Someone's already connected to the romi
- Someone (100% not Ibrahim) drove the romi off a table
- Other Reasons (Please ask experienced member)

### Possible fixes
- Turn the romi on (Try both the switch and the button)
- Ask if other people are connected to the romi
- Put batteries/replace the batteries in the romi
- Restart your computer

## Sim interface

<img src="./Sim.png" alt="Sim Interface" width="1301"/>

### Robot State
This is the current state of the robot. The robot can be:

- Disconnected
- Disabled
- Autonomus
- Teleoperated

Disconnected: The robot is not receiving any instructions from the computer.
Disabled: It is communicating with the computer, but is unable to run any code.
Autonomus: It will run code that is preloaded. The human cannot control the robot.
Teleoperated: The human will give the robot inputs, which it will run.
### Joysticks

### Other
You will almost never need to touch the other sim elements, such as Timing and Network tables. If it is necessary, an experienced member will tell you.

## Conventions
### Naming Conventions
On ChainLynx, we use the following naming conventions

```java
// For classes
public class RobotContainer {}

// For objects,
private Subsystem elevatorSubsystem;

// For constants,

public static final double kMaxVelocity;

// For class fields,

private double speedMultiplier;
```


### Units
[The Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html) allows you to store physical measurements such as distance or angle rather than just storing a number. This can help eliminate mistakes, if for example you were given a number and you thought it was in pounds but it was in kilograms.