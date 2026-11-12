// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

/** Numbers the whole robot uses. Nothing functional goes in here. */
public final class Constants {
  /**
   * Using a real Romi? Don't change this here: set {@code useRealRomi} at the top of build.gradle.
   */
  public static final boolean kUseRealRomi = "true".equals(System.getenv("USE_REAL_ROMI"));
}
