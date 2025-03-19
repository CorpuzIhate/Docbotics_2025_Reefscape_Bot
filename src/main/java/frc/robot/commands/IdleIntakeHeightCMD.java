package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSub;

public class IdleIntakeHeightCMD extends Command {
    /**Add subsystem,motors and PIDController */
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    public final SparkMax rightElevatorMotor;
    public final PIDController elevatorController;

    /**
     * 
     * @param elevatorSub elevator subsystem.
     */
    public IdleIntakeHeightCMD(
        /**Constructor for subsystem, motors, PIDController */
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
        /**Set command to running on the dashboard */
        SmartDashboard.putBoolean("IdleIntakeHeightCMD", true);

    }

    @Override
    public void execute() {
        //Telemetery
        /**PID Values to the dashboard */
        SmartDashboard.putData("elevatorController", elevatorController);
        /**Intake Position to the dashboard */
        SmartDashboard.putNumber("intakeHeightSetPoint", elevatorSub.getIntakeHeightSetPoint_Inches());
        /**Error Position to the dashboard */
        SmartDashboard.putNumber("elevatorPositionError_Inches", elevatorController.getError());
        /**Elevator Position to the dashboard */
        SmartDashboard.putNumber("elevatorPosition_Inches", elevatorSub.getPrimaryElevatorPosition());
        // Drive elevator Motor to set-point based on elevator controller.
        //TODO AFTER TESTING CHANGE SETPOINT TO THE VARIABLE SETPOINT IN ELEVATORSUB. 
        double output = elevatorController.calculate(elevatorSub.getPrimaryElevatorPosition(), elevatorSub.getIntakeHeightSetPoint_Inches());
        primaryLeftElevatorMotor.set(output);
    }

    
    @Override
    public void end(boolean interrupted) {
        /**Set command to NOT running on the dashboard */
        SmartDashboard.putBoolean("IdleIntakeHeightCMD", false);
            /**
         * When elevator command ends
         * stop all motors;
         */
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}