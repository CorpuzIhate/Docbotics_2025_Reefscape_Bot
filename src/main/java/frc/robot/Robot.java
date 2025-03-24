// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.LimelightHelpers;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  





  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // logger code 

    Logger.recordMetadata("ProjectName", "6110_SwerveTest_2024_02_20"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    CameraServer.startAutomaticCapture();



    m_robotContainer = new RobotContainer();


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("elevatorPosition_Inches", m_robotContainer.
    elevatorSub.getPrimaryElevatorPosition());
    SmartDashboard.putNumber("elevatorAppliedVoltage", m_robotContainer.
    elevatorSub.getPrimaryLeftElevatorMotor().getAppliedOutput());
    SmartDashboard.putNumber("intakePitchPosition_degrees", m_robotContainer.coralPitcherIntakeSub.getIntakePitcherMotor().getAbsoluteEncoder().getPosition());

    SmartDashboard.putNumber("intakePitchAppliedVoltage", m_robotContainer.coralPitcherIntakeSub.getIntakePitcherMotor().getAppliedOutput());
    SmartDashboard.putNumber("IntakeSetpoint_degrees", m_robotContainer.coralPitcherIntakeSub.getIntakePitchSetpoint_degrees()
   );
   SmartDashboard.putNumber("intakeHeightSetPoint", m_robotContainer.
   elevatorSub.getIntakeHeightSetPoint_Inches());


   SmartDashboard.putNumber("BotPoseX",LimelightHelpers.getBotPose3d_TargetSpace("").getX());

   SmartDashboard.putNumber("BotPoseY",LimelightHelpers.getBotPose3d_TargetSpace("").getY());
   SmartDashboard.putNumber("BotPoseZ",-LimelightHelpers.getBotPose3d_TargetSpace("").getZ());

   SmartDashboard.putBoolean("isYPosOrientedToTarget",m_robotContainer.limelightSub.isYPosOrientedToTarget(
    -LimelightHelpers.getBotPose3d_TargetSpace("").getZ()
   ));
   SmartDashboard.putBoolean("isXPosOrientedToTarget",m_robotContainer.limelightSub.isXPosOrientedToTarget(
    LimelightHelpers.getBotPose3d_TargetSpace("").getX()
   ));



    
    CommandScheduler.getInstance().run();

    
  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    SmartDashboard.putString("Auto on?", "yes");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic(){

    

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.swerveSub.stopModules();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}