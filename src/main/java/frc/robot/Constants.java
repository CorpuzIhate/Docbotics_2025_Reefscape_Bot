// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants of the individual swerve modules. */
  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 

    //Drive and turning gear ratios.   
    public static final double kDriveMotorGearRatio = 1 / 6.75 ;
    public static final double kTurningMotorGearRatio =  1 / 12.8;

    // Conversion factors for drive motor's position and velocity.
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

    // Conversion factors for turn motor's position and velocity.
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // The propertional coefficent for the turning PID controller.
    public static final double kTurningControllerPValue = 0.25; 
    
  }

  /**Constansts related to the drivetrain. */
   public static final class DriveConstants {

        //Swerve kinematics constants. Used in swerve subsystem to convert desired chassis speeds
        //into the individual motor speeds for each swerve modules.
        public static final double kTrackWidth = Units.inchesToMeters(19.75); //changed.
        // Distance between right and left wheels.
        public static final double kWheelBase = Units.inchesToMeters(26.5); //changed.
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
        //CAN ID's of the absolute encoders of the swerve modules.

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 20;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 21;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 19;
        public static final int kBackRightDriveAbsoluteEncoderPort = 22;

        //Encoder reversal configurations.
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


        /** Absolute encoder offsets of the swerve module. This is set by aligning the swerve
         * modules so that they are  parrelle and straight, with flat side out for better allignment.
         * These values can be see on the Phoenix tuner and are given in rotation, so we convert to radians,
         */
        public static final class DriveAbsoluteEncoderOffsetRad{
          public static final double kFrontLeft = 0.35864 * 2 * Math.PI ;
          public static final double kBackLeft = 0.23388 * 2 * Math.PI;
          public static final double kFrontRight = -0.2861 * 2 * Math.PI;
          public static final double kBackRight = 0.08715 * 2 * Math.PI ;
        }
        /** Max speed of the drive motors in meters per second. Used in both swerve module class and 
         * swerve subsystem to limit drive motor speeds.
         */
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        /** Max speed of the turning motors in radians  per second. 
         * Used in orient to target function to limit turning speeds and in
         * kTeleDriveMaxAngularSpeedRadiansPerSecond to define the max tele-op turning speed,  */
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        /**Max tele-op drive speed (m/s).*/
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3;
        /**Max tele-op turning speed (rad/s).*/
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        /** Max tele-op drive acceleration(m/s^2). */
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        /**Max tele-op turning acceleration (rad/s^2).*/
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /** Constants use in auto targeting in swerve subsystem. */
        public static class autoTargetConstants{
          /** Propertional coeffiencent of the auto targeting turning 
           * PID controller. */
          public static final double autoOrientKp = 0.0035;
          

        }
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        /* ID of the joystick axis's. Used in SwerveJoystick Command
          to determine which direction an input in certain joystick axis's 
          will go.*/
        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 4;
  
        /** Configure robot into field oriented mode button. */
        public static final int kDriverFieldOrientedButtonIdx = 1;
        /**Orient to Target button. B button. */
        public static final int kOrientToTargetIdx = 2; 
        /**reset gyro buttton. */
        public static final int kDriveGyroResetButtonIdx = 2;
      
        /** Move Arm Button. A button. */
        public static final int kMoveArmIdx  = 1; 


        /** deadband of the joystick when driving in tele-op.
        Prevents small changes in the joystick from moving the
        robot.
        */
        public static final double kDeadband = 0.5;


    }
        //Constants for the arm.
    public static final class ArmConstants {
      // CAN ID of arm motor.
      public static final int kArmMotorPort = 13;
      //Arm PID controller constants.
      public static final double kP = 0.00175;
      public static final double kI = 0;
      public static final double kD = 0.0000525;
      
    } 

    //Constants for Auto.
    public static final class AutoConstants {

      /**Registered commands. Used in Robotcontainer to register commands into
       * Pathplanner so they can be activated during auto.
       */
      public static final class autoCommands{
        public static final String moveArmCMD ="moveArmCMD";
      }

  }

}