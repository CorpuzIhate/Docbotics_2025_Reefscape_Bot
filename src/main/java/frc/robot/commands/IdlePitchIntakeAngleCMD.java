package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralPitcherIntakeSub;

public class IdlePitchIntakeAngleCMD extends Command {
    /**Add subsystem, motor, PID Controller */
    private final CoralPitcherIntakeSub intakeSub;
    private final SparkMax intakePitcherMotor;
    private final PIDController intakePitchController;


    public IdlePitchIntakeAngleCMD(CoralPitcherIntakeSub intakeSub) {
        /**Constructor for subsystem, motors and PIDcontrollers*/
        this.intakeSub = intakeSub;
        this.intakePitcherMotor = intakeSub.getIntakePitcherMotor();
        this.intakePitchController = intakeSub.getIntakePitchController();
        
        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake pitcher */
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();

       
        /**Set command to running on the dashboard */
        SmartDashboard.putBoolean("isIdleIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        double currentIntakePosition_degrees = intakePitcherMotor.getAbsoluteEncoder().getPosition();
        // Telemetry.
        /**PID Values to the dashboard */
        SmartDashboard.putData("intakePitcherController",intakePitchController);
        
        /**Error Position to the dashboard */
        SmartDashboard.putNumber("intakePitchPositionError_degrees", intakePitchController.getError());
        
        /**Actual Position to the dashboard */
        SmartDashboard.putNumber("intakePitchPosition_degrees", currentIntakePosition_degrees);
        
        // Pitches intake to set point based on intake pitch controller.
        double output = intakePitchController.calculate(currentIntakePosition_degrees, intakeSub.getIntakePitchSetpoint_degrees());
       /**double Output to the dashboard */
        SmartDashboard.putNumber("intakePitcherOutput", output);
        intakePitcherMotor.set(output);
    }

    @Override
    public void end(boolean interrupted) {
        /**set command to NOT running on dashboard */
        SmartDashboard.putBoolean("isIdleIntakePitcherCommandRunning", false);
        /* when command ends, stop  the intake pitch motor. */
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}