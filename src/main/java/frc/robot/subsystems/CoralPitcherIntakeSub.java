
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralPitcherIntakeSub extends SubsystemBase {
    /**Motor that rotates the intake. */
    private SparkMax intakePitcherMotor = new SparkMax(IntakeConstants.kIntakePitcherMotorPort, MotorType.kBrushless);

    /**Motor controller configuratiion of the intake pitcher motor. */
    private SparkMaxConfig intakePitcherMotorConfig = new SparkMaxConfig();
    /**PID controller to rotate the intake to an angular setpoint */
    private PIDController intakePitchController = new PIDController(
            IntakeConstants.intakePitcher_kP,
            IntakeConstants.intakePitcher_kI,
            IntakeConstants.intakePitcher_kD);
    /**angular set point of the pitcher in degrees. */
    private double pitcherSetpoint_degrees = 10;
    public CoralPitcherIntakeSub(){
        intakePitchController.enableContinuousInput(0, 360);

        intakePitcherMotorConfig.absoluteEncoder.positionConversionFactor(IntakeConstants.intakePitcherRotationsToDegrees);
        intakePitcherMotor.configure(intakePitcherMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
    /**Sets the angular setpoint of the intake in degrees. */
    public void setIntakePitchSetpoint_degrees(double setpoint_degrees){
        pitcherSetpoint_degrees = setpoint_degrees;
    }
    /**@return the angular setpoint of the intake in degrees. */
    public double getIntakePitchSetpoint_degrees( ){
        return pitcherSetpoint_degrees;
    }

}