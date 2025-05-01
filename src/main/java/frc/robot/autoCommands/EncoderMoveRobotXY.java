package frc.robot.autoCommands;


import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSub;


public class EncoderMoveRobotXY extends Command {
    SwerveSub swerveSub;
    Translation2d currentRobotPoseToDesiredRobotPose;

    /*displacement from robot starting pose to current pose */
    Translation2d startToCurrentRobotPose;
    /*displacement from robot starting pose to desired pose */
    Translation2d startToDesiredRobotPose;  
    /*Rotation2d at the start of the command. */
    Rotation2d initialRotation2d;


    /*Pid controllers  that control the speed of 
    robot as is appraoches its disired position */
    PIDController xTranslationXContoller = 
    new PIDController(0.5, 0, 0);
    
    PIDController yTranslationXContoller = 
    new PIDController(0.5, 0, 0);
    PIDController thetaTranslationXContoller = 
    new PIDController(0.1, 0, 0);

/**moves robot based on desired displacements from current position, x+  forward, y+ left */
    public EncoderMoveRobotXY(
        SwerveSub swerveSub,
        Translation2d currentRobotPoseToDesiredRobotPose
    ) {
        this.swerveSub = swerveSub;
        this.currentRobotPoseToDesiredRobotPose = currentRobotPoseToDesiredRobotPose;
        addRequirements(swerveSub);

    }

    @Override
    public void initialize() {
        /*Takes the current position of the robot. */
        startToCurrentRobotPose = swerveSub.getPose().getTranslation();
        initialRotation2d = swerveSub.getRotation2d();
        startToDesiredRobotPose = startToCurrentRobotPose.plus(currentRobotPoseToDesiredRobotPose);
        
        xTranslationXContoller.setTolerance(0.15);
        yTranslationXContoller.setTolerance(0.15);

        SmartDashboard.putNumber("InitalPoseX",startToCurrentRobotPose.getX());
        SmartDashboard.putNumber("InitalPoseY",startToCurrentRobotPose.getY());

        SmartDashboard.putNumber("CurrentPoseX",startToCurrentRobotPose.getX());
        SmartDashboard.putNumber("CurrentPoseY",startToCurrentRobotPose.getY());
        SmartDashboard.putNumber("DesiredPosetX()",startToDesiredRobotPose.getX());
        SmartDashboard.putNumber("DesiredPoseY()",startToDesiredRobotPose.getY());
    }

    @Override
    public void execute() {
    /*Takes the current position of the robot. */
    startToCurrentRobotPose = swerveSub.getPose().getTranslation();
    Rotation2d currentRotation2d = swerveSub.getRotation2d();
    SmartDashboard.putNumber("CurrentPoseX",startToCurrentRobotPose.getX());
    SmartDashboard.putNumber("CurrentPoseY",startToCurrentRobotPose.getY());

    /*sets the of velocity of the robot depending on the how close it is to desired position. */
    double xOutput = xTranslationXContoller.calculate(startToCurrentRobotPose.getX(),startToDesiredRobotPose.getX());
    double yOutput = yTranslationXContoller.calculate(startToCurrentRobotPose.getY(),startToDesiredRobotPose.getY());
    /*theta controller keeps the robot at the same heading as it translates */
    double thetaOutput = thetaTranslationXContoller.calculate(currentRotation2d.getDegrees(),initialRotation2d.getDegrees());
    /*if positon controller is at setpoint, set its output to zero. */
    if(xTranslationXContoller.atSetpoint()){
        xOutput = 0;
    }
    if(yTranslationXContoller.atSetpoint()){
        yOutput = 0;
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput,thetaOutput );
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    /*apply chassis speeds to robot. */
    swerveSub.setModuleStates(moduleStates, false);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        /*if the robot is at the desired position, end the command. */
        if(xTranslationXContoller.atSetpoint() && yTranslationXContoller.atSetpoint())
        {
            return true;
        }
        return false;
    }
}