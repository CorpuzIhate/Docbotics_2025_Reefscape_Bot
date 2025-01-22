package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSub;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MoveArmCMD extends Command{
    public final ArmSub armSub; 
    public final SparkMax armMotor;
    public final PIDController armController;


    
    public MoveArmCMD(ArmSub armSub){
        this.armSub = armSub;
        this.armMotor = armSub.getMotor();
        armController = armSub.getArmController();
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        armMotor.set(0);
        armMotor.stopMotor();
        
        SmartDashboard.putBoolean("isArmCommandRunning", true);
    }

    
    @Override
    public void execute(){
        //telemetry
        SmartDashboard.putData(armController);
        SmartDashboard.putNumber("armPostionError_degrees",armController.getError());
        SmartDashboard.putNumber("armPostion_degrees",armSub.getGetArmEncoderPosition_degrees());
        //drive arm Motor to setpoint based on arm controller
        double output = armController.calculate(armSub.getGetArmEncoderPosition_degrees(), 169);
        armMotor.set(output);       
    }
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("isArmCommandRunning", false);
        armMotor.set(0);
        armMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        
        return false;
    }
}