
package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReefTagCMD extends Command {
    private final SwerveSub m_swerveSub;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
 
    public AlignToReefTagCMD(SwerveSub swerveSub) {
        m_swerveSub = swerveSub;
        /*initialize PID Chassis position controllers. */
        m_xController = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController);
        m_yController = new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController);
        m_rotationController = new PIDController(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController);

        m_rotationController.enableContinuousInput(-180, 180); // important for heading



        addRequirements(swerveSub);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();

    }

    @Override
    public void execute() {



        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");

        /*
         * x -> forward/ back from the robot
         * y -> left / right from the robot
         * 
         */
 
        
        /**gets the current heading of the robot in degrees. The is from -180 to 180 */
        double currentHeading = m_swerveSub.getHeading();
        /**sets the desired gyro heading */

        double desiredHeading = 0; //FIX ME
        
        /**gets left and right distance in meters from A-tag. */
        double yDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace("").getX();
        /**gets forward and right distance in meters from A-tag. */
        double xDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace("").getZ();

        /**Sends telemetry related to the alignment with the reef branch. */
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        SmartDashboard.putData("limeLight_xController", m_xController);
        SmartDashboard.putData("limelight_yController", m_yController);
        SmartDashboard.putData("limelight_thetaController",m_rotationController);

        // Calculate PID outputs.
        /*makes the chassis go look directly at A-tag be, have no left or right 
         * distance from the A-tag, with a 1.1 m distance from the front bumper of 
         * the robot.
          */
        double xOutput = m_xController.calculate(xDistanceFromTarget_meters, 1.1); 
        double yOutput = m_yController.calculate(yDistanceFromTarget_meters, 0); 
        double rotationOutput = m_rotationController.calculate(currentHeading, desiredHeading); 

        /**applies a deadband to motor outputs.*/
        if (Math.abs(xOutput) < 0.05){
            xOutput = 0;
        }
        if (Math.abs(rotationOutput) < 0.05){
            rotationOutput = 0;
        }

        if (Math.abs(yOutput) < 0.05){
            yOutput = 0;
        }
        // Create chassis speeds and drive.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, -yOutput, rotationOutput);
        /*Convert chassis speeds into module states */
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        /**Sends motor power output telemetry to SmartDashboard.  */
        SmartDashboard.putNumber("rotationOutput", rotationOutput);
        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);
        /**applies chassisSpeeds to modules. */
        m_swerveSub.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("running?",false);

        m_swerveSub.stopModules();

    }

    @Override
    public boolean isFinished() {
        if(!LimelightHelpers.getTV(""))
        {
            // No target found, stop.
            return true;
        }
        /*if we're close enough to being aligned with the 
         * april tag, STOP.
         */
        if(m_xController.getError() < 0.05 &&
         m_yController.getError() < 0.05 &&
        m_rotationController.getError() < 0.05)
        {
            return true;
        }
        // You might want to add a tolerance check here to stop when close enough to the target.
        return false; // Run until interrupted by driver.
    }
}
