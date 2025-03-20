package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSub;

public class IdleIntakeHeightCMD extends Command {
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    
    public final SparkMax rightElevatorMotor;
    public final PIDController elevatorController;

    /**
     * 
     * @param elevatorSub elevator subsystem.
     */
    public IdleIntakeHeightCMD(
            ElevatorSub elevatorSub) {
        this.elevatorSub = elevatorSub;
        this.primaryLeftElevatorMotor = elevatorSub.getPrimaryLeftElevatorMotor();
        this.rightElevatorMotor = elevatorSub.getRightElevatorMotor();
        this.elevatorController = elevatorSub.getElevatorController();
        addRequirements(elevatorSub);

    }

    @Override
    public void initialize() {

        /**
         * When command starts, stop all elevator motors.
         * Right elevator motor will always FOLLOW
         * the left elevator motor in the OPPOSITE direction.
         */
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();

        SmartDashboard.putBoolean("IdleIntakeHeightCMD", true);

    }

    @Override
    public void execute() {
        /* Send elevator telemetry */
        SmartDashboard.putData("elevatorController", elevatorController);
        SmartDashboard.putNumber("intakeHeightSetPoint", elevatorSub.getIntakeHeightSetPoint_Inches());
        SmartDashboard.putNumber("elevatorPositionError_Inches", elevatorController.getError());
        SmartDashboard.putNumber("elevatorPosition_Inches", elevatorSub.getPrimaryElevatorPosition());
        // Drive elevator Motor to set-point based on elevator controller.
        //AFTER TESTING CHANGE SETPOINT TO THE VARIABLE SETPOINT IN ELEVATORSUB. 
        double output = elevatorController.calculate(elevatorSub.getPrimaryElevatorPosition(), elevatorSub.getIntakeHeightSetPoint_Inches());
        primaryLeftElevatorMotor.set(output);
    }

    /**
     * When elevator command ends
     * stop all motors;
     */
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("IdleIntakeHeightCMD", false);
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}