package frc.robot.commands;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DismountSub;

public class MoveDismountArmCMD extends Command{
    private final DismountSub dismountSub; 
    private final SparkMax dismountArmMotor;
    private final PIDController dismountController;
    private final double setpoint_degrees;

    
    public MoveDismountArmCMD(DismountSub dismountSub, double setpoint_degrees){
        this.dismountSub = dismountSub;
        this.dismountArmMotor = dismountSub.getDismountArmMotor();
        dismountController = dismountSub.getDismountController();

        this.setpoint_degrees = setpoint_degrees;
        addRequirements(dismountSub);
        
    }

    @Override
    public void initialize(){
        
        /**Stops all dismount motor movement at the beginning of command */
        dismountArmMotor.set(0);
        dismountArmMotor.stopMotor();
        
        SmartDashboard.putBoolean("isdismountArmCommandRunning", true);
    }

    
    @Override
    public void execute(){
        /**current dismount arm angle in degrees. */
        double currentDismountArmPos_degrees = dismountArmMotor.getAbsoluteEncoder().getPosition();
        //telemetry
        SmartDashboard.putData(dismountController);
        SmartDashboard.putNumber("dismountPositionError_degrees",dismountController.getError());
        SmartDashboard.putNumber("dismountPosition_degrees",currentDismountArmPos_degrees);
        //drive arm Motor to setpoint based on arm controller
        double output = dismountController.calculate(currentDismountArmPos_degrees, setpoint_degrees);
        dismountArmMotor.set(output);       
    }
    @Override
    public void end(boolean interrupted){
   /**Stops all dismount motor movement at the end of command */

        SmartDashboard.putBoolean("isdismountArmCommandRunning", false);
        dismountArmMotor.set(0);
        dismountArmMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        

        /*  If we want dismount arm to maintain its angular positon,
         * continue powering the arm.
        */
        if(dismountSub.getIsHoldPosition()){
            return false;
        }
        //if the dismount arm motor is within 0.5 degrees of the setpoint end the command 
        if(Math.abs(dismountController.getError()) <= 0.5){
            return true;
        }
        return false;
    }
}