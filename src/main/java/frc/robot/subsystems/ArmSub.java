
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSub extends SubsystemBase {
    /** Motor of the arm. */
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    /** PID controller of the arm. */
    private PIDController armController = new PIDController(
            ArmConstants.kP,
            ArmConstants.kI,
            ArmConstants.kD);

    public ArmSub() {

    }

    /**
     * @return motor of the arm.
     */
    public SparkMax getMotor() {
        return armMotor;
    }

    /**
     * @return returns the arm encoder position in degrees.
     */
    public double getGetArmEncoderPosition_degrees() {
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    /**
     * @return the PID controller of the arm.
     *
     */
    public PIDController getArmController() {
        return armController;
    }

    /**
     * sets the arm Motor power.
     * 
     * @param power
     *            percentage of motor power from -1 to 1
     */
    public void setArmMotorPower(double power) {
        armMotor.set(power);
    }

}