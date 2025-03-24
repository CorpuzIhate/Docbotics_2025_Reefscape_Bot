// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.DismountConstants;
import frc.robot.autoCommands.autoPowerCoralIntakeCMD;
import frc.robot.autoCommands.resetSwerveModuleSpeedsCMD;
import frc.robot.commands.ElevateIntakeToSetpointCMD;
import frc.robot.commands.IdleIntakeHeightCMD;
import frc.robot.commands.IdlePitchIntakeAngleCMD;
import frc.robot.commands.powerCoralIntakeCMD;
import frc.robot.commands.powerDismountSpinMotorCMD;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.MoveDismountArmCMD;
import frc.robot.commands.PitchIntakeCMD;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.SwerveJoystickCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;
import frc.robot.subsystems.DismountSpinSub;
import frc.robot.subsystems.DismountSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OIConstants.kDriverControllerPort);

  public final SwerveSub swerveSub = new SwerveSub();
  public final CoralPitcherIntakeSub coralPitcherIntakeSub = new CoralPitcherIntakeSub();
  private final CoralIntakeConsumerSub coralIntakeConsumerSub = new CoralIntakeConsumerSub();
  // private final ArmSub armsub = new ArmSub();
  public final LimelightSub limelightSub = new LimelightSub();
  public final ElevatorSub elevatorSub = new ElevatorSub();
  private final DismountSub dismountSub = new DismountSub();
  private final DismountSpinSub dismountSpinSub = new DismountSpinSub();


  private final SendableChooser<Command> autoChooser;
  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);
  

  public RobotContainer() {

    // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kSlowModeIdx),
             /// By default will be on field oriented.
            () -> !
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
            () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx))); 
    limelightSub.setDefaultCommand(
        new ManageLimeLightCMD(limelightSub));

    dismountSub.setDefaultCommand(
      new MoveDismountArmCMD(dismountSub)
    );

    
    /**
     * By default the the elevator will be in Idle state
     * where it just tries to maintain the intake height set point.
     */
     elevatorSub.setDefaultCommand(
     new IdleIntakeHeightCMD(elevatorSub));

     
     coralPitcherIntakeSub.setDefaultCommand(
      new IdlePitchIntakeAngleCMD(coralPitcherIntakeSub)
     );
     
     coralIntakeConsumerSub.setDefaultCommand(
      new powerCoralIntakeCMD(
        coralIntakeConsumerSub, 
        () -> driverJoyStick.getRawAxis(OIConstants.kIntakeAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kOutakeAxis) ));

    configureBindings();

      autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    // new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new
    // MoveArmCMD(armsub));


  

    /**Command to reset intake elevator motors */
    Command resetEncodersCommand = new RunCommand(() -> {
      elevatorSub.resetElevatorEncoders();
    }, elevatorSub);
    SmartDashboard.putData("resetEncodersCommand", resetEncodersCommand);


    Command consumerCoralAtCoralStation = new ParallelCommandGroup(
      new InstantCommand(() -> 
      {elevatorSub.setIntakeHeightSetPoint_Inches(30.8); 
      coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(144);}));
      NamedCommands.registerCommand("consumeCoralAtCoralStation", consumerCoralAtCoralStation); 

    Command scoreL2Reef = new ParallelCommandGroup(
      new InstantCommand(() -> 
      {elevatorSub.setIntakeHeightSetPoint_Inches(43);
      coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(91);}));
      NamedCommands.registerCommand("scoreL2Reef", scoreL2Reef);

    Command scoreL3Reef = new ParallelCommandGroup(
        new InstantCommand(() -> 
        {elevatorSub.setIntakeHeightSetPoint_Inches(84);
        coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(118);})); 
        NamedCommands.registerCommand("scoreL3Reef", scoreL3Reef);

    Command setIntakePositionToDefault = new ParallelCommandGroup(
      new InstantCommand(() -> 
      {elevatorSub.setIntakeHeightSetPoint_Inches(0);
      coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(60);}));  

      Command scoreL1Reef  = new ParallelCommandGroup(
        new InstantCommand(() -> 
        {elevatorSub.setIntakeHeightSetPoint_Inches(0);
        coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(100);}));  

      NamedCommands.registerCommand("setIntakePositionToDefault", setIntakePositionToDefault);
      NamedCommands.registerCommand("intake", new autoPowerCoralIntakeCMD(coralIntakeConsumerSub, -0.3).withTimeout(3));
      

      NamedCommands.registerCommand("outtake", new autoPowerCoralIntakeCMD(coralIntakeConsumerSub, 0.3).withTimeout(0.5));
      NamedCommands.registerCommand("resetSwerveModuleSpeedsCMD", new resetSwerveModuleSpeedsCMD(swerveSub));
      new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToDefaultPosIdx).
    onTrue(setIntakePositionToDefault);
    /**When button pressed moved Intake to reef level 2 height and angle. */  
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel2Idx).
    onTrue(scoreL2Reef);
    
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel3Idx).
    onTrue(scoreL3Reef);
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToCoralStationIdx).
    onTrue(consumerCoralAtCoralStation);

    /**When button pressed reset the gyro. */
    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

    /**Command that dismounts Algea from the reef. */
    Command dismountAlgeaL2CMD = 
    new SequentialCommandGroup(
      new InstantCommand(() -> { dismountSub.setIsHoldPosition(!dismountSub.getIsHoldPosition());}),
      new ConditionalCommand(
        new InstantCommand(() -> {
          dismountSub.setSetpoint(DismountConstants.dismountAlegeSetpointL2_degrees);
          dismountSpinSub.getDismountSpinMotor().set(0.6);}),
        new InstantCommand(() -> {
          dismountSub.setSetpoint(0);
          dismountSpinSub.getDismountSpinMotor().set(0);}), 
        ()-> { return dismountSub.getIsHoldPosition();})
    );
    Command dismountAlgeaL3CMD = 
    new SequentialCommandGroup(
      new InstantCommand(() -> { dismountSub.setIsHoldPosition(!dismountSub.getIsHoldPosition());}),
      new ConditionalCommand(
        new InstantCommand(() -> {
          dismountSub.setSetpoint(DismountConstants.dismountAlegeSetpointL3_degrees);
          dismountSpinSub.getDismountSpinMotor().set(0.6);}),
        new InstantCommand(() -> {
          dismountSub.setSetpoint(0);
          dismountSpinSub.getDismountSpinMotor().set(0);}), 
        ()-> { return dismountSub.getIsHoldPosition();})
    );

    Command autoDismountAlgeaL3CMD = 
    new InstantCommand(() ->  {
      dismountSub.setIsHoldPosition(true);
      dismountSpinSub.getDismountSpinMotor().set(0.6);
      dismountSub.setSetpoint(DismountConstants.dismountAlegeSetpointL3_degrees);});
    Command autoDismountAlgeaL2CMD = 
    new InstantCommand(() ->  {
      dismountSub.setIsHoldPosition(true);
      dismountSpinSub.getDismountSpinMotor().set(0.6);
      dismountSub.setSetpoint(DismountConstants.dismountAlegeSetpointL2_degrees);});
    Command autoDismountDefaultCMD = 
    new InstantCommand(() ->  {
      dismountSub.setIsHoldPosition(true);
      dismountSpinSub.getDismountSpinMotor().set(0.6);
      dismountSub.setSetpoint(0);});

    /*checks whether up on the d-pad is pressed. */
    Trigger isDpadUpPressed = new Trigger(() -> {return driverJoyStick.getPOV() == 0;});
    Trigger isDpadRightPressed = new Trigger(() -> {return driverJoyStick.getPOV() == 90;});
    Trigger isDpadLeftPressed = new Trigger(() -> {return driverJoyStick.getPOV() == 270;});

    /**dismount Algea when up on the d-pad is pressed. */


    isDpadUpPressed.onTrue(dismountAlgeaL3CMD);

    isDpadLeftPressed.onTrue(dismountAlgeaL2CMD);

    SmartDashboard.putData("MoveForward" ,new PathPlannerAuto("MoveForward"));
    
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }
}