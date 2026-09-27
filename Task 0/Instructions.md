# Task 0: Configuring Computer

## Overview

It is recommended to bring your own laptop as school laptops have limitations. We have 2 drive laptops that can sometimes be used for coding, but usually they are being used to test the robot.

Installing WPILib VScode is recommended unless you have a good reason not to. Normal VScode will not use the wpilib JDK by default and is missing all of the preinstalled extensions as well and more.

## Downloading WPILib

Go to [https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html#downloading], and download the installer.

<img src="./Download.png" alt="Download button" width="901"/>

Then follow the install instructions below the download button.

## Downloading & configuring Git

### Windows

If you use a package manager like [winget](https://learn.microsoft.com/en-us/windows/package-manager/) or [chocolatey](https://chocolatey.org/) use those, otherwise go to [https://git-scm.com/install/windows] and download and run the standalone installer.

### MacOS

If you already have homebrew then use `brew install git`, otherwise use `xcode-select --install` to install the xcode command line tools which comes with git.

### Linux

Its probably already installed, you can check with `which git` if not use `<your favorate package manager invocation to install packages> git`.

### Configuration

In order to use git you will need set some option, all should be set with `git config --global <the setting>`

- Your name. `user.name=<your name>` You should generally set this to the same thing as your forgejo username and beware that this will be public
- An email. `user.email=<your username>@noreply.localhost`

There a few more settings you should set

- `rerere.enabled=true`
- `push.autosetupremote=true`
- `pull.rebase=true`
- `pull.merge=false`
- `rebase.autostash=true`

## Connecting to the Romis

<img src="./RomiPowerButton.png" alt="Top view of a romi with the power switch circled in red" width=500>

First power on the romi by flipping the power switch to the on position. Once you are done with the romi flip the switch to the off position and then press the button next to the switch. If you here the romi beeping that means it is low on battery.

Once the romi is on you can connect to the romis, by disconnecting from your current wi-fi network and find the romi's network(either romi-1 or romi-2 the number is on the bottom), the password will be set to `82488248`. If you can't find it, here are some common causes:

- Romi is off
- Romi is out of battery
- You are attempting to connect to the wrong romi
- You have wi-fi disabled
- Someone's already connected to the romi

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

[The Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html) allows you to store physical measurements such as distance or angle rather than just storing a number. This can help eliminate mistakes, if for example you were given a number and you thought it was in pounds but it was in kilograms. Anytime a value that can be represented as a unit is used it should be.
