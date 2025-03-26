package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.SwerveSub;

public class LockWheelsCMD extends Command{
    private final SwerveSub swerveSub; 
    private final SwerveModuleState[] desiredLockOnStates 
    = new SwerveModuleState[]{

        new SwerveModuleState(0, new Rotation2d(-0.394* 2 * Math.PI)), // front right
        new SwerveModuleState(0, new Rotation2d(0.489 * 2 * Math.PI)), // front left

        new SwerveModuleState(0, new Rotation2d(0.25 * 2 * Math.PI)),// back right

        new SwerveModuleState(0, new Rotation2d(0.1246 * 2 * Math.PI)) // back left

    };




    
    public LockWheelsCMD(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        
    }

    @Override
    public void initialize(){
        swerveSub.stopModules();
    }

    
    @Override
    public void execute(){
        swerveSub.setModuleStates(desiredLockOnStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSub.stopModules();

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}