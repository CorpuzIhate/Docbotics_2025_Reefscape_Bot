package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.SwerveSub;

public class LockWheelsCMD extends Command{
    private final SwerveSub swerveSub; 
    public final Supplier<Double> xSpdFunction;
    public final Supplier<Double> ySpdFunction;
    public final Supplier<Double> turningSpdFunction;

    private final SwerveModuleState[] desiredLockOnStates 
    
    = new SwerveModuleState[]{

        new SwerveModuleState(0, new Rotation2d (- Math.PI/4 )), // front right
        new SwerveModuleState(0, new Rotation2d( Math.PI /4)), // front left

        new SwerveModuleState(0, new Rotation2d (Math.PI /4)),// back right

        new SwerveModuleState(0, new Rotation2d( -Math.PI /4)) // back left

    };




    
    public LockWheelsCMD(
        SwerveSub swerveSub,
        Supplier <Double> xSpdFunction,
        Supplier<Double> ySpdFunction, 
        Supplier<Double> turningSpdFunction){
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;

        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
        
    }

    @Override
    public void initialize(){
        swerveSub.stopModules();
    }

    
    @Override
    public void execute(){
        swerveSub.setModuleStates(desiredLockOnStates, false);
    }

    @Override
    public void end(boolean interrupted){
        swerveSub.stopModules();

    }
    @Override
    public boolean isFinished(){
        double xspeed = xSpdFunction.get();
        double yspeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        
        //now apply deband,  if joystick doesnt center back to exactly zero, it still stops
        xspeed = Math.abs(xspeed) > OIConstants.kDeadband ? xspeed : 0.0;
        yspeed = Math.abs(yspeed) > OIConstants.kDeadband ? yspeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
    
        if( Math.abs(xspeed) > 0 || Math.abs(yspeed) > 0 || Math.abs(turningSpeed) > 0){
          return true;
        }
        return false;
    }
}