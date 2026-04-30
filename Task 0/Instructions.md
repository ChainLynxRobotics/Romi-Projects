# Task 0: Configuring Computer

## Overview

It is recommended to bring your own laptop as school laptops have limitations. We have 2 drive laptops that can sometimes be used for coding, but usually they are being used to test the robot.

Installing WPILib VScode is very necessary. Normal VScode will won't work, as it is missing critical tools required to run and simulate the robot.

## Downloading WPILib

Go to https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html, scroll down, and download the installer.

<img src="./Download.png" alt="Download button" width="901"/>

Then follow the install instructions below the download button.

## Downloading & configuring Git

Git is also a necessary for uploading your code to the hub. Go to https://git-scm.com/install/ and follow the install instructions in the page.

Next go to https://git-scm.com/book/ms/v2/Getting-Started-First-Time-Git-Setup to configure your user profile in order to begin using git.
**Whatever you set as your git name and email will be public on github** If you dont want your actual email to be public you can use [githubs private email](https://docs.github.com/en/account-and-profile/how-tos/email-preferences/setting-your-commit-email-address)

There are a few settings that you should enable once you set your name and email.
- rerere.enabled true
- push.autosetupremote true
- pull.rebsse true
- pull.merge false
- rebase.autostash

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
[The Units library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html) allows you to store physical measurements such as distance or angle rather than just storing a number. This can help eliminate mistakes, if for example you were given a number and you thought it was in pounds but it was in kilograms. This can happen to even expericenced coders such as those at [NASA](https://en.wikipedia.org/wiki/Mars_Climate_Orbiter), so it's importiant to always use the units library where applicable.

