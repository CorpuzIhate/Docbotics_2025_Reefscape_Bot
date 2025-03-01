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

    
    public MoveDismountArmCMD(DismountSub dismountSub){
        this.dismountSub = dismountSub;
        this.dismountArmMotor = dismountSub.getDismountArmMotor();
        dismountController = dismountSub.getDismountController();

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
        if(!dismountSub.getIsHoldPosition()){
            
            dismountArmMotor.set(0);  
            return;     

        }
        double output = dismountController.calculate(currentDismountArmPos_degrees, dismountSub.getSetpoint());
        
        /*If the subsystem wants to hold its position,
         * power the dismount arm, if not, disable the arm.
         */

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
        


        return false;
    }
}