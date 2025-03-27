// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.SwerveSub;

 

public class SwerveJoystickCmd extends Command {

      private final SwerveSub swerveSubsystem;


      public final Supplier<Double> xSpdFunction;
      public final Supplier<Double> ySpdFunction;
      public final Supplier<Double> turningSpdFunction;
      public final Supplier<Boolean> fieldOrientedFunction;
      private final Supplier<Boolean> slowModeFunction;
      public final Supplier<Boolean> lockWheelsFunction;
      private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; // slew rate limiter cap the the amount of change of a value

      private boolean isSlowMode;
      public static double CurrentXSpeed;
      public static double CurrentYSpeed;
      public static double CurrentTurningSpeed;
      public static boolean CurrentOrientation;
      private final SwerveModuleState[] desiredLockOnStates 

    = new SwerveModuleState[]{

        new SwerveModuleState(0, new Rotation2d(-0.394* 2 * Math.PI)), // front right
        new SwerveModuleState(0, new Rotation2d(-0.489 * 2 * Math.PI)), // front left
        new SwerveModuleState(0, new Rotation2d(0.25 * 2 * Math.PI)),// back right
        new SwerveModuleState(0, new Rotation2d(0.1246 * 2 * Math.PI)) // back left

    };
      
  public SwerveJoystickCmd(
          SwerveSub swerveSubsystem, 
          Supplier <Double> xSpdFunction,
           Supplier<Double> ySpdFunction, 
           Supplier<Double> turningSpdFunction,
           Supplier<Boolean> slowModeFunction,
          Supplier<Boolean> fieldOrientedFunction,
          Supplier<Boolean> lockWheelsFunction) { // Supplier<Boolean> limeTargetAccessed//
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.slowModeFunction = slowModeFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.lockWheelsFunction = lockWheelsFunction;


        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lockWheelsFunction.get())
    {
      swerveSubsystem.setModuleStates(desiredLockOnStates, false);

      return;
    }
    // gett latest values from joystick
    //swerveSubsystem.orientToTarget();
    double xspeed = xSpdFunction.get();
    double yspeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    
    //now apply deband,  if joystick doesnt center back to exactly zero, it still stops
    xspeed = Math.abs(xspeed) > OIConstants.kDeadband ? xspeed : 0.0;
    yspeed = Math.abs(yspeed) > OIConstants.kDeadband ? yspeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // allows for violent joystick movements to be more smooth

    xspeed = xLimiter.calculate(xspeed) *  DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    yspeed = yLimiter.calculate(yspeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
    turningSpeed = turningLimiter.calculate(turningSpeed) *
     DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    //If slow mode toggle is on apply it to the modules.
    if(slowModeFunction.get()){
        isSlowMode = !isSlowMode;
    }
    if(isSlowMode){
      xspeed *= 0.35;
      yspeed *= 0.35;
      turningSpeed *= 0.35;
    }
    

    //select orintatin of robot

    ;
    ChassisSpeeds chassisSpeeds;
 
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xspeed, -yspeed, -turningSpeed, swerveSubsystem.getRotation2d());


    SmartDashboard.putBoolean("targetOn", lockWheelsFunction.get());
    
    CurrentXSpeed = xspeed;
    CurrentYSpeed = yspeed;
    CurrentTurningSpeed = turningSpeed;
    CurrentOrientation = fieldOrientedFunction.get();



    // convert chassis speeds to individual module states; later to switch to velocity
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    // set state to each wheel


    swerveSubsystem.setModuleStates(moduleStates, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}