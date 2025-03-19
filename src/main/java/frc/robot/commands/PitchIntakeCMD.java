package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralPitcherIntakeSub;

public class PitchIntakeCMD extends Command {
    private final CoralPitcherIntakeSub intakeSub;
    private final SparkMax intakePitcherMotor;
    private final PIDController intakePitchController;
    private final double setpoint_degrees;

    public PitchIntakeCMD(CoralPitcherIntakeSub intakeSub, double setpoint_degrees) {
        this.intakeSub = intakeSub;
        this.intakePitcherMotor = intakeSub.getIntakePitcherMotor();
        this.intakePitchController = intakeSub.getIntakePitchController();
        this.setpoint_degrees = setpoint_degrees;


        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake pitcher */
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();
        intakeSub.setIntakePitchSetpoint_degrees(setpoint_degrees);
        

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        /**Gets the current position of the intake pitcher motor. */
        double currentIntakePosition_degrees = intakePitcherMotor.getAbsoluteEncoder().getPosition();
        // Telemetry.
        SmartDashboard.putData(intakePitchController);
        SmartDashboard.putNumber("intakePitchPositionError_degrees", intakePitchController.getError());
        SmartDashboard.putNumber("intakePitchPosition_degrees", currentIntakePosition_degrees);

        // Pitches intake to set point based on intake pitch controller.
        double output = intakePitchController.calculate(currentIntakePosition_degrees, setpoint_degrees);
        SmartDashboard.putNumber("intakePitcherOutput", output);
        intakePitcherMotor.set(output);
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake pitch motor. */
        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", false);
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}