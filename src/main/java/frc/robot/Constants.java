// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.undo.StateEdit;

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

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kTurning = 0.25; // changed from 0.5

  }

  public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(19.75); //changed
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(28.6); //changed 2025 01 25
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 5;
    public static final int kBackLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 10;
    public static final int kBackLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 4;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 21;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 19;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.35864 * 2 * Math.PI;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.23388 * 2 * Math.PI;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.2861 * 2 * Math.PI;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.08715 * 2 * Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond  / 1.6;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static class autoTargetConstants {
      public static final double autoOrientKp = 0.0035;

    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverXAxis = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverRotAxis = 4;
    public static final int kIntakeAxis = 3;
    public static final int kOutakeAxis = 2;

        public static final int kDriverFieldOrientedButtonIdx = 3;
        public static final int kOrientToTargetIdx = 2; //B button 
        public static final int kDriveGyroResetButtonIdx = 7;
      

            // **Button that powers coral intake. */
    public static final int kIntakeCoralIdx = 5;
                // **Button that powers coral intake. */
                public static final int kOutakeCoralIdx = 6;
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
    public static final int kSlowModeIdx = 4;


    public static final double kDeadband = 0.5;

  }

  public static final class ArmConstants {
    public static final int kArmMotorPort = 13;
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
    /**Converts revolutions of the elevator motor's encoder to gear revolutions. */
    public static double elevatorMotorEncoderRevToGearRev = 1/ 20;
    public static double elevatorSprocketPitchDiameter_inches = 1.751;
    /** converts elevator gear revolutions to linear motion in inches*/
    public static double ElevatorGearRevToLinearMotion_Inches = elevatorSprocketPitchDiameter_inches * Math.PI;
    /**
     * Conversion from rotation of the primary elevator motor
     * to meters. Used for getting current position of the tallest point on the
     * to the ground
     */
    public static final double elevatorMotorRotationToMeters = elevatorMotorEncoderRevToGearRev * ElevatorGearRevToLinearMotion_Inches;

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
    /**Converts rotations of the intake pitcher motor to degrees */
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
      public static final double maxDismountPower =0.5;

      public static final double dismountAlegeSetpointL2_degrees = 37;

      public static final double dismountAlegeSetpointL3_degrees = 75;
    }
      
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 6;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;
    
    public static final double kIXController = 0;
    public static final double kDXController = 0;
    public static final double kIYController = 0;
    public static final double kDYController = 0;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final class autoCommands {
      public static final String moveArmCMD = "moveArmCMD";
    }



  }
  public static final class LimeLightConstants{
    public static final double kBranchTargetYOffset = 0;
    public static double kLeftBranchTargetXOffset = 0;



  }

  public static class IndexerConstants {
    public static final int kIndexerPWMPort = 2;

  }
}