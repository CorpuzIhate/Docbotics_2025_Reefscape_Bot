
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



public class DismountSub extends SubsystemBase{
    /** Motor that pitches the dismount arm.*/
    private SparkMax dismountArmMotor = new SparkMax(DismountConstants.kDismountArmMotorPort, MotorType.kBrushless);
    /** Motor thhat spins the dismount motor.*/

    private SparkMaxConfig dismountArmMotorConfig = new SparkMaxConfig();
    /**state to determine whether the dismount should hold its position */
    private boolean isHoldPosition;
        /**PID controller that moves the dismount 
     * arm motor to its angular set points degrees. */
    private PIDController dismountController= new PIDController(
        DismountConstants.dismount_kP,
        DismountConstants.dismount_kI,
        DismountConstants.dismount_kD);

    private double setpoint_degrees;


    
    public DismountSub(){
        /*because the dismount subsystem is a anglar PID controller, enable continoue input makes
         * 0 and 360 degrees the same set point.
         */
        dismountController.enableContinuousInput(0, 360);

        /*sets dismount arm encoder units.  */
        dismountArmMotorConfig.absoluteEncoder.positionConversionFactor(360);
        /*applies dimount arm motor configuration to the dismount arm motor. */
        dismountArmMotor.configure(dismountArmMotorConfig,ResetMode.kResetSafeParameters,
         PersistMode.kNoPersistParameters);

        

        
    }



    /**@return the motor that pitches the dismount arm. */
    public SparkMax getDismountArmMotor(){
        return dismountArmMotor;
    }
    /**@return the motor thhat spins the dismount motor. */


    /**@return PID controller that moves the dismount 
     * arm motor to its angular set points degrees.
     */
    public PIDController getDismountController(){
        return dismountController;
    }
    /**@return whether the dismount arm should
     * maintain is angular position.
     */
    public boolean getIsHoldPosition(){
        return isHoldPosition;
    }
        /**@return whether the dismount arm should
     * maintain is angular position.
     */
    public void setIsHoldPosition(boolean state){
        isHoldPosition = state;
    }

    public double getSetpoint(){
        return setpoint_degrees;
    }

    public void setSetpoint(double newSetpoint_degrees){
        setpoint_degrees = newSetpoint_degrees;
    }




}