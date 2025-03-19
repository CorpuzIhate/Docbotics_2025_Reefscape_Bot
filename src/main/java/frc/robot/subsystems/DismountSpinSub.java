
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.DismountConstants;


import com.revrobotics.spark.SparkLowLevel.MotorType;




public class DismountSpinSub extends SubsystemBase{
    /** Motor thhat spins the dismount motor.*/
    private SparkMax dismountSpinMotor = new SparkMax(DismountConstants.kDismountSpinMotorPort, MotorType.kBrushless);



    





    /**@return the motor thhat spins the dismount motor. */

    public SparkMax getDismountSpinMotor(){
        return dismountSpinMotor;
    }
    /**@return PID controller that moves the dismount 
     * arm motor to its angular set points degrees.
     */





}