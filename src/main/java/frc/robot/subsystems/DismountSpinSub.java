
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.google.flatbuffers.Constants;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants.DismountConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;



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