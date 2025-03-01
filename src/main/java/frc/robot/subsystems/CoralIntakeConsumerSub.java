
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntakeConsumerSub extends SubsystemBase {

    /**Motor that allows the intake to consume coral. */
    private SparkMax intakeConsumerMotor = new SparkMax(IntakeConstants.kIntakeConsumerMotorPort, MotorType.kBrushless);
    
    /**
     * @return the intake motor that consumes the Coral.
     */
    public SparkMax getIntakeConsumerMotor() {
        return intakeConsumerMotor;
    }





}