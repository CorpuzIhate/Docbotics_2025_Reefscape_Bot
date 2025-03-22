// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants.DismountConstants;
import frc.robot.autoCommands.autoPowerCoralIntakeCMD;
import frc.robot.autoCommands.resetSwerveModuleSpeedsCMD;
import frc.robot.commands.AlignToBranchCMD;
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

  // configures the different subsystem of the robot,
  public final SwerveSub swerveSub = new SwerveSub();
  public final CoralPitcherIntakeSub coralPitcherIntakeSub = new CoralPitcherIntakeSub();
  private final CoralIntakeConsumerSub coralIntakeConsumerSub = new CoralIntakeConsumerSub();
  private final LimelightSub limelightSub = new LimelightSub();
  public final ElevatorSub elevatorSub = new ElevatorSub();
  private final DismountSub dismountSub = new DismountSub();
  private final DismountSpinSub dismountSpinSub = new DismountSpinSub();

  private final SendableChooser<Command> autoChooser;
  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);
  /* D-pad Triggers. */
  private final Trigger isDpadUpPressed = new Trigger(() -> {
    return driverJoyStick.getPOV() == 0;
  });
  private final Trigger isDpadRightPressed = new Trigger(() -> {
    return driverJoyStick.getPOV() == 90;
  });
  private final Trigger isDpadLeftPressed = new Trigger(() -> {
    return driverJoyStick.getPOV() == 270;
  });

  public RobotContainer() {

    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kSlowModeIdx),
            /// By default will be on field oriented.
            () -> !driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
            () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx)));

    limelightSub.setDefaultCommand(
        new ManageLimeLightCMD(limelightSub));

    dismountSub.setDefaultCommand(
        new MoveDismountArmCMD(dismountSub));

    /**
     * By default the the elevator will be in Idle state
     * where it just tries to maintain the intake height set point.
     */
    elevatorSub.setDefaultCommand(
        new IdleIntakeHeightCMD(elevatorSub));

    coralPitcherIntakeSub.setDefaultCommand(
        new IdlePitchIntakeAngleCMD(coralPitcherIntakeSub));

    coralIntakeConsumerSub.setDefaultCommand(
        new powerCoralIntakeCMD(
            coralIntakeConsumerSub,
            () -> driverJoyStick.getRawAxis(OIConstants.kIntakeAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kOuttakeAxis)));

    configureBindings();
    /* add PathPlanner autos as options in auto selector. */
    SmartDashboard.putData("Center_1Coral_F2_Reef", new PathPlannerAuto("Center_1Coral_F2_Reef"));
    SmartDashboard.putData("Center_1Coral_I2_CoralStation", new PathPlannerAuto("Center_1Coral_I2_CoralStation"));
    SmartDashboard.putData("Center_1Coral_I2_Reef", new PathPlannerAuto("Center_1Coral_I2_Reef"));
    SmartDashboard.putData("MoveForward", new PathPlannerAuto("MoveForward"));
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Configure the trigger bindings for certain commands
   * and registers them in PathPlanner
   */
  private void configureBindings() {

    /** Command to resets elevator encoders */
    Command resetElevatorEncoderCommand = new RunCommand(() -> {
      elevatorSub.resetElevatorEncoders();
    }, elevatorSub);

    /*
     * creates a button on SmartDashboard
     * for resetting the Encoders.
     */
    SmartDashboard.putData("resetEncodersCommand", resetElevatorEncoderCommand);

    /** Command to get intake to height and angular coral station set-point. */
    Command consumerCoralAtCoralStation = new ParallelCommandGroup(

        new InstantCommand(() -> {
          elevatorSub.setIntakeHeightSetPoint_Inches(Constants.ElevatorConstants.elevatorSetpoint.coralStationSetpoint_inches);
          coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(Constants.IntakeConstants.IntakePitchSetPoints_degrees.coralStation_degrees);
        }));

    /* registers the command in PathPlanner. */
    NamedCommands.registerCommand("consumeCoralAtCoralStation", consumerCoralAtCoralStation);

    /* binds Command to trigger. */
    
     new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToCoralStationIdx).onTrue(consumerCoralAtCoralStation);

    /** Command to get intake to height and angular level 2 reef set-point. */
    Command scoreL2Reef = new ParallelCommandGroup(

        new InstantCommand(() -> {
          elevatorSub.setIntakeHeightSetPoint_Inches(Constants.ElevatorConstants.elevatorSetpoint.reefLevel2Setpoint_inches);
          coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(Constants.IntakeConstants.IntakePitchSetPoints_degrees.L2Pitch_degrees);
        }));

    /* registers the command in PathPlanner. */
    NamedCommands.registerCommand("scoreL2Reef", scoreL2Reef);


    /** binds Command to trigger. */
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel2Idx).onTrue(scoreL2Reef);
    new JoystickButton(driverJoyStick, OIConstants.kOrientToTargetIdx).whileTrue(
      new AlignToBranchCMD(swerveSub, () -> driverJoyStick.getRawButton(Constants.OIConstants.kAlignLeft))
    );
    //
    /** Command to get intake to height and angular level 3 reef set-point. */
    Command scoreL3Reef = new ParallelCommandGroup(

        new InstantCommand(() -> {
          elevatorSub.setIntakeHeightSetPoint_Inches(Constants.ElevatorConstants.elevatorSetpoint.reefLevel3Setpoint_inches);
          coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(Constants.IntakeConstants.IntakePitchSetPoints_degrees.L3Pitch_degrees);
        }));
    /* registers the command in PathPlanner. */


    NamedCommands.registerCommand("scoreL3Reef", scoreL3Reef);
    /** binds Command to trigger. */
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel3Idx).onTrue(scoreL3Reef);

    /** Command to get intake to height and angular default set-point. */
    Command setIntakePositionToDefault = new ParallelCommandGroup(
        new InstantCommand(() -> {
          elevatorSub.setIntakeHeightSetPoint_Inches(0);
          coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(60);
        }));
    /* registers the command in PathPlanner. */
    NamedCommands.registerCommand("setIntakePositionToDefault", setIntakePositionToDefault);

    /** binds Command to trigger. */
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToDefaultPosIdx).onTrue(setIntakePositionToDefault);

    /** Command to get intake to height and angular level 1 reef set-point. */
    Command scoreL1Reef = new ParallelCommandGroup(
        new InstantCommand(() -> {
          elevatorSub.setIntakeHeightSetPoint_Inches(0);
          coralPitcherIntakeSub.setIntakePitchSetpoint_degrees(100);
        }));

    NamedCommands.registerCommand("intake", new autoPowerCoralIntakeCMD(coralIntakeConsumerSub, -0.3).withTimeout(3));
    NamedCommands.registerCommand("outtake", new autoPowerCoralIntakeCMD(coralIntakeConsumerSub, 0.3).withTimeout(0.5));
    NamedCommands.registerCommand("resetSwerveModuleSpeedsCMD", new resetSwerveModuleSpeedsCMD(swerveSub));

    /** When button pressed reset the gyro. */
    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
        new ResetHeadingCMD(swerveSub));

    /** Command that dismounts algae from reef level 2. */
    Command dismountAlgaeL2CMD = new SequentialCommandGroup(
        new InstantCommand(() -> {
          dismountSub.setIsHoldPosition(!dismountSub.getIsHoldPosition());
        }),
        new ConditionalCommand(
            new InstantCommand(() -> {
              dismountSub.setSetpoint(DismountConstants.dismountAlgaeSetPointL2_degrees);
              dismountSpinSub.getDismountSpinMotor().set(0.6);
            }),
            new InstantCommand(() -> {
              dismountSub.setSetpoint(0);
              dismountSpinSub.getDismountSpinMotor().set(0);
            }),
            () -> {
              return dismountSub.getIsHoldPosition();
            }));
    /** binds Command to trigger. */
    isDpadLeftPressed.onTrue(dismountAlgaeL2CMD);
    /** Command that dismounts Algae from reef level 3. */

    Command dismountAlgaeL3CMD = new SequentialCommandGroup(
        new InstantCommand(() -> {
          dismountSub.setIsHoldPosition(!dismountSub.getIsHoldPosition());
        }),
        new ConditionalCommand(
            new InstantCommand(() -> {
              dismountSub.setSetpoint(DismountConstants.dismountAlgaeSetPointL3_degrees);
              dismountSpinSub.getDismountSpinMotor().set(0.6);
            }),
            new InstantCommand(() -> {
              dismountSub.setSetpoint(0);
              dismountSpinSub.getDismountSpinMotor().set(0);
            }),
            () -> {
              return dismountSub.getIsHoldPosition();
            }));
    /** binds Command to trigger. */

    isDpadUpPressed.onTrue(dismountAlgaeL3CMD);

  }

  /* registers commands into PathPlanner. */
  public void configureNamedCommands() {

  }

  // ** gets the autonomous Command for the robot */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

  }

}