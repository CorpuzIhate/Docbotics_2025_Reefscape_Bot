// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants of the individual swerve modules. */
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    // Drive and turning gear ratios.
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    // Conversion factors for drive motor's position and velocity.
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

    // Conversion factors for turn motor's position and velocity.
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // The propertional coefficent for the turning PID controller.
    public static final double kTurningControllerPValue = 0.25;

  }

  /** Constansts related to the drivetrain. */
  public static final class DriveConstants {

    // Swerve kinematics constants. Used in swerve subsystem to convert desired
    // chassis speeds
    // into the individual motor speeds for each swerve modules.
    public static final double kTrackWidth = Units.inchesToMeters(19.75); // changed.
    // Distance between right and left wheels.
    public static final double kWheelBase = Units.inchesToMeters(26.5); // changed.
    // Distance between front and back wheels.
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // CAN ID's of the different drive and turning motors.
    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 10;
    public static final int kBackLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 4;
    // CAN ID's of the absolute encoders of the swerve modules.

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 21;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 19;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    // Encoder reversal configurations.
    public static final boolean kIsFrontLeftTurningEncoderReversed = true;
    public static final boolean kIsBackLeftTurningEncoderReversed = true;
    public static final boolean kIsFrontRightTurningEncoderReversed = true;
    public static final boolean kIsBackRightTurningEncoderReversed = true;

    public static final boolean kIsFrontLeftDriveEncoderReversed = false;
    public static final boolean kIsBackLeftDriveEncoderReversed = false;
    public static final boolean kIsFrontRightDriveEncoderReversed = true;
    public static final boolean kIsBackRightDriveEncoderReversed = true;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    /**
     * Absolute encoder offsets of the swerve module. This is set by aligning the
     * swerve
     * modules so that they are parrelle and straight, with flat side out for better
     * allignment.
     * These values can be see on the Phoenix tuner and are given in rotation, so we
     * convert to radians,
     */
    public static final class DriveAbsoluteEncoderOffsetRad {
      public static final double kFrontLeft = 0.35864 * 2 * Math.PI;
      public static final double kBackLeft = 0.23388 * 2 * Math.PI;
      public static final double kFrontRight = -0.2861 * 2 * Math.PI;
      public static final double kBackRight = 0.08715 * 2 * Math.PI;
    }


    /**
     * Max speed of the drive motors in meters per second. Used in both swerve
     * module class and
     * swerve subsystem to limit drive motor speeds.
     */
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    /**
     * Max speed of the turning motors in radians per second.
     * Used in orient to target function to limit turning speeds and in
     * kTeleDriveMaxAngularSpeedRadiansPerSecond to define the max tele-op turning
     * speed,
     */
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    /** Max tele-op drive speed (m/s). */
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3;
    /** Max tele-op turning speed (rad/s). */
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    /** Max tele-op drive acceleration(m/s^2). */
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    /** Max tele-op turning acceleration (rad/s^2). */
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    /** Constants use in auto targeting in swerve subsystem. */
    public static class autoTargetConstants {
      /**
       * Propertional coeffiencent of the auto targeting turning
       * PID controller.
       */
      public static final double autoOrientKp = 0.0035;

    }
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;
    /*
     * ID of the joystick axis's. Used in SwerveJoystick Command
     * to determine which direction an input in certain joystick axis's
     * will go.
     */
    public static final int kDriverXAxis = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverRotAxis = 4;

    public static int kIntakeAxis = 3;
    public static int kOuttakeAxis = 2;

    /** Configure robot into field oriented mode button. */
    public static final int kDriverFieldOrientedButtonIdx = 1;
    /** Orient to Target button. B button. */
    public static final int kOrientToTargetIdx = 2;
    /** reset gyro buttton. */
    public static final int kDriveGyroResetButtonIdx = 7;

    /** Move Arm Button. A button. */
    public static final int kMoveArmIdx = 1;
    /** Button if the robot will be in slow mode. */
    public static final int kSlowModeIdx = 4;

    // *Button that moves intake to default height and angle. */
    public static final int kMoveIntakeToDefaultPosIdx = 1;

    /**
     * Button ID to move the intake to the angle and height for
     * score level 2 of the reef.
     */
    public static final int kMoveIntakeToLevel2Idx = 2;

    /**
     * Button ID to move the elevator to level 3 on the Reef.
     * (A button)
     */
    public static final int kMoveIntakeToLevel3Idx = 6;

    public static final int kMoveIntakeToCoralStationIdx = 5;
    /**
     * deadband of the joystick when driving in tele-op.
     * Prevents small changes in the joystick from moving the
     * robot.
     */
    public static final double kDeadband = 0.5;

  }

  // Constants for the arm.
  public static final class ArmConstants {
    // CAN ID of arm motor.
    public static final int kArmMotorPort = 13;
    // Arm PID controller constants.
    public static final double kP = 0.00175;
    public static final double kI = 0;
    public static final double kD = 0.0000525;

  }

  /** Constants for the elevator. (TEMPORARY, NEEDS TO ALL BE TUNED) */
  public static final class ElevatorConstants {
    /* Motor ports for the elevator. */
    public static final int kLeftElevatorMotorPort = 9;
    public static final int kRightElevatorMotorPort = 11;
    /** PID coefficients for elevator controller. */
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0.0000525;
    /** Initial height of the intake to the ground in meters. */
    public static final double initialHeightOfIntakeToGround_Meters = 101.1;
    /** Converts revolutions of the elevator motor's encoder to gear revolutions. */
    public static double elevatorMotorEncoderRevToGearRev = 1 / 20;
    public static double elevatorSprocketPitchDiameter_inches = 1.751;
    /** converts elevator gear revolutions to linear motion in inches */
    public static double ElevatorGearRevToLinearMotion_Inches = elevatorSprocketPitchDiameter_inches * Math.PI;
    /**
     * Conversion from rotation of the primary elevator motor
     * to meters. Used for getting current position of the tallest point on the
     * to the ground
     */
    public static final double elevatorMotorRotationToMeters = elevatorMotorEncoderRevToGearRev
        * ElevatorGearRevToLinearMotion_Inches;

    /** Minimum height the Intake relative to the ground. */
    public static final double minIntakeHeightToGround_Meters = 0;

    /**
     * Used in slew rate limiter in JoystickMoveIntakeCMD in
     * order to smooth the change in joystick input.
     */
    public static final double elevatorJoystickSensitivity = 0.1;
    /**
     * Used in JoystickMoveIntakeCMD
     * to keep small input in the joystick
     * from moving the intake.
     */
    public static final double elevatorJoystickDeadband = 0.1;

    /** Elevator setpoints from the ground */
    public static final class elevatorSetpoint {

      public final static double reefLevel2Setpoint_inches = 0;
      public final static double reefLevel3Setpoint_inches = 0;
      public final static double coralStationSetpoint_inches = 30.8;
    }

  }

  public static final class IntakeConstants {
    /** CAN ID of the intake motor that consumes the coral. */
    public static final int kIntakeConsumerMotorPort = 13;
    /** CAN ID of the intake motor pitches the intake. */
    public static final int kIntakePitcherMotorPort = 16;
    /** Converts rotations of the intake pitcher motor to degrees */
    public static final double intakePitcherRotationsToDegrees = 360;
    /* PID coefficients of the intake pitcher controller. */

    public static final double intakePitcher_kP = 0.008;
    public static final double intakePitcher_kI = 0;
    public static final double intakePitcher_kD = 0.0000525;

    /** angular set points of the intake pitcher in degrees. */
    public static final class IntakePitchSetPoints_degrees {
      public static final double coralStation_degrees = 144;
      public static final double L2Pitch_degrees = 0;
      public static final double L3Pitch_degrees = 0;
    }

    public static final class DismountConstants {
      /** CAN ID of the intake motor that dismounts the algea. */
      public static final int kDismountSpinMotorPort = 15;
      /** CAN ID of the intake motor pitches the dismount. */
      public static final int kDismountArmMotorPort = 17;
      /* PID coefficients of the intake pitcher controller. */

      public static final double dismount_kP = 0.012;
      public static final double dismount_kI = 0;
      public static final double dismount_kD = 0.0000525;
      public static final double maxDismountPower = 0.5;

      public static final double dismountAlgaeSetPointL2_degrees = 37;

      public static final double dismountAlgaeSetPointL3_degrees = 75;
    }

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 6;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;


    public static final double kPXController = 0.01;
    public static final double kPYController = 0.01;
    public static final double kPThetaController = 0.1;

    public static double kIXController = 0;
    public static double kIYController = 0;
    public static double kIThetaController = 0;
    public static double kDXController = 0;
    public static double kDYController = 0;
    public static double kDThetaController = 0;
    public static double kLeftBranchTargetXOffset = 0;
    public static double kBranchTargetYOffset = 0;
    public static double kLeftBranchTargetRotationOffset = 0;
    public static double kRightBranchTargetXOffset = 0;
    public static double kRightBranchTargetRotationOffset = 0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final class autoCommands {
      public static final String moveArmCMD = "moveArmCMD";
    }



  }

  public static final class LimelightConstants{
    /**angle of limelight from the ground in degrees. */
    public static double kLimeLightAngleFromGround_degrees = 0;
    /**height of limelight from ground in inches. */
    public static double limelightHeight_inches = 12.625;
    /*height of the target from ground in inches. */
    public static double reefTargetHeight_Inches = 11.125;

    

  }

}