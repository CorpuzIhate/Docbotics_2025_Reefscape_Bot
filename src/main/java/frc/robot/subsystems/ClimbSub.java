
package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;



public class ClimbSub extends SubsystemBase{
    /*Climb Motors. */
    private SparkMax primaryLeftClimbMotor = new SparkMax(ClimbConstants.kPrimaryLeftClimbMotorPort, MotorType.kBrushless);
    private SparkMax rightClimbMotor = new SparkMax(ClimbConstants.kRightClimbMotorPort, MotorType.kBrushless);
    
    /*Climb Motor Configurations */
    private SparkMaxConfig primaryLeftClimbMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();


    /**Climb motor controller. */    
    private PIDController climbController= new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD);

    public ClimbSub(){
        /*right climb motor will mirror the left climb motor's movement.  */
        rightClimbMotorConfig.
        follow(ClimbConstants.kPrimaryLeftClimbMotorPort, true);
        /*converts rotations of the climb motor to degrees */
        primaryLeftClimbMotorConfig.absoluteEncoder.positionConversionFactor(360);

        /*apply motor configs */
        primaryLeftClimbMotor.configure(primaryLeftClimbMotorConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

        rightClimbMotor.configure(rightClimbMotorConfig, ResetMode.kNoResetSafeParameters, 
        PersistMode.kNoPersistParameters);
    }
    
    



    /**@return get primary climber motor.  */
    public SparkMax getPrimaryClimbMotor(){
        return primaryLeftClimbMotor;
    }
    /**@return climber motor encoder position in degrees. All positions are based on left climb motor*/
    public double getGetClimbEncoderPosition_degrees(){
        return primaryLeftClimbMotor.getAbsoluteEncoder().getPosition();
    }
    /**@return get the climb motor controller. */
    public PIDController getClimbController(){
        return climbController;
    }
    /**@return set climber motor power. */
    public void setClimbMotorPower(double power){
        primaryLeftClimbMotor.set(power);
    }





}