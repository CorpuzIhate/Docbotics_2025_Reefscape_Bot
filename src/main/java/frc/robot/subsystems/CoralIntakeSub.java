
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntakeSub extends SubsystemBase {
    private SparkMax intakePitcherMotor = new SparkMax(IntakeConstants.kIntakePitcherMotorPort, MotorType.kBrushless);
    private SparkMax intakeConsumerMotor = new SparkMax(IntakeConstants.kIntakeConsumerMotorPort, MotorType.kBrushless);

    private SparkMaxConfig intakePitcherMotorConfig = new SparkMaxConfig();
    private PIDController intakePitchController = new PIDController(
            IntakeConstants.intakePitcher_kP,
            IntakeConstants.intakePitcher_kI,
            IntakeConstants.intakePitcher_kD);
    public CoralIntakeSub(){
        intakePitcherMotorConfig.absoluteEncoder.positionConversionFactor(IntakeConstants.intakePitcherRotationsToDegrees);
        intakePitcherMotor.configure(intakePitcherMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    /**
     * @return the intake motor that consumes the Coral.
     */
    public SparkMax getIntakeConsumerMotor() {
        return intakeConsumerMotor;
    }

    /**
     * @return the motor that pitches the intake.
     */
    public SparkMax getIntakePitcherMotor() {
        return intakePitcherMotor;
    }

    /** @return the PID controller of the intake pitcher */
    public PIDController getIntakePitchController() {
        return intakePitchController;
    }

}