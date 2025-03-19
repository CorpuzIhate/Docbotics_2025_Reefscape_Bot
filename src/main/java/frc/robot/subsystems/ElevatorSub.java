
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSub extends SubsystemBase {
    /* Elevator motors. */
    private SparkMax primaryLeftElevatorMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorPort,
            MotorType.kBrushless);
    private SparkMax rightElevatorMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorPort, MotorType.kBrushless);
    /* Elevator motor configurations. */
    private SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig primaryLeftElevatorMotorConfig = new SparkMaxConfig();

    /** PID controller to move the elevator move to position. */
    private PIDController elevatorPIDController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD);
    private double intakeSetpointFromGround_Meters = ElevatorConstants.minIntakeHeightToGround_Meters;
    public ElevatorSub() {
        /**
         * Right elevator motor will follow the
         * left motor and be inverted.
         */
        rightElevatorMotorConfig
                .follow(ElevatorConstants.kLeftElevatorMotorPort, true);
        /**
         * Convert primary elevator motor encoder ticks to
         * distance of the intake to the ground in meters.
         */
        primaryLeftElevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.elevatorMotorRotationToMeters);

        /**
         * apply right and left elevator motor config. Will only change new parameters.
         * parameters do no persist across power cycles.
         */
        rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

    }

    /** @return the primary left elevator Motor. */
    public SparkMax getPrimaryLeftElevatorMotor() {
        return primaryLeftElevatorMotor;
    }

    /** @return the right elevator Motor. */
    public SparkMax getRightElevatorMotor() {
        return rightElevatorMotor;
    }

    /** @return primary elevator Motor position in meters. */
    public double getPrimaryElevatorPosition() {
        return primaryLeftElevatorMotor.getEncoder().getPosition();
    }

    /** @return the elevator PID Controller */
    public PIDController getElevatorController() {
        return elevatorPIDController;
    }
    /**@return the height set-point of the intake in Inches   */
    public double getIntakeHeightSetPoint_Inches(){
        return intakeSetpointFromGround_Meters;
    }
    /**
     * Sets the height set point of the intake in meters.
     * @param heightSetPoint_Inches height set point of the intake in meters. 
     */
    public void setIntakeHeightSetPoint_Inches(double heightSetPoint_Inches){
        intakeSetpointFromGround_Meters = heightSetPoint_Inches;
    }
    /**resets the encoders current position */
    public void resetElevatorEncoders(){
        primaryLeftElevatorMotor.getEncoder().setPosition(0);
    }
}