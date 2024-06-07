// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  public static final double kRobotMass = 40; // [kg]
  public static final Matter kChassis = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass);
  public static final double kLoopTime = 0.13; // s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID = new PIDConstants(0.5, 0, 0.01);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double kWheelLockTime = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double kLeftXDeadband = 0.1;
    public static final double kLeftYDeadband = 0.1;
    public static final double kRightXDeadband = 0.1;
    public static final double kTurnSpeed = 150;
  }
}
