
package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlignToReefTagCMD extends Command {
    private final SwerveSub m_swerveSub;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    private final String limelightName = "limelight-left";
    private final Supplier <Double> m_leftTriggerInput;
    private double desiredYDistanceFromTarget_meters;

    private boolean isXOutputDisabled;
    private boolean isYOutputDisabled;


    private final SlewRateLimiter xLimiter  = new SlewRateLimiter(0.65,-1,0);
 
    public AlignToReefTagCMD(SwerveSub swerveSub ,Supplier <Double> leftTriggerInput) {
        m_swerveSub = swerveSub;
        /*initialize PID Chassis position controllers. */
        m_leftTriggerInput = leftTriggerInput;
        m_xController = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController);
        m_yController = new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController);
        m_rotationController = new PIDController(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController);

        m_rotationController.enableContinuousInput(-180, 180); // important for heading



        addRequirements(swerveSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("ALign_running?",true);
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();
        desiredYDistanceFromTarget_meters = 0.5;

        isXOutputDisabled = false;
        isYOutputDisabled = false;

        m_xController.setTolerance(0.0075);


        m_yController.setTolerance(0.35);
        m_yController.setP(Constants.AutoConstants.kPYController);


        m_rotationController.setTolerance(0.2);
        SmartDashboard.putNumber("m_leftTriggerInput", m_leftTriggerInput.get());
        if( m_leftTriggerInput.get() >= 0.5 ){
            desiredYDistanceFromTarget_meters *= -1;
        }

    }

    @Override
    public void execute() {



        

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
        double yDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getX();
        /**gets forward and backward distance in meters from A-tag. */
        double xDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace(limelightName).getZ();

        /**Sends telemetry related to the alignment with the reef branch. */
     
        SmartDashboard.putNumber("botPose_FwBw",xDistanceFromTarget_meters);
        SmartDashboard.putNumber("botPose_LR", yDistanceFromTarget_meters   );

        
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        SmartDashboard.putData("limeLight_xController", m_xController);
        SmartDashboard.putData("limelight_yController", m_yController);
        SmartDashboard.putData("limelight_thetaController",m_rotationController);
        SmartDashboard.putNumber("x_Error", m_xController.getError());
        SmartDashboard.putNumber("y_Error", m_yController.getError());
        SmartDashboard.putNumber("turnign_error",m_rotationController.getError());

        // Calculate PID outputs.
        /*makes the chassis go look directly at A-tag be, have no left or right 
         * distance from the A-tag, with a 1.1 m distance from the front bumper of 
         * the robot.
          */
          double xOutput = 0;
          double yOutput = 0;
        if(!isXOutputDisabled){
            xOutput = m_xController.calculate(xDistanceFromTarget_meters, -1); 

        }
        if(!isYOutputDisabled){
            yOutput = m_yController.calculate(yDistanceFromTarget_meters, desiredYDistanceFromTarget_meters); 
        }
        /*face directly towards the april tag at all times */
        double rotationOutput = m_rotationController.calculate(LimelightHelpers.getTX(limelightName), 0); 
        
        xOutput = xLimiter.calculate(xOutput);
      

        /**If the the robot is at x set-point,
         * disabled x output, and increase the p value of the 
         * ycontroller to align the robot to its y target.
         */
        if (m_xController.atSetpoint()){ 

            isXOutputDisabled = true;
            m_yController.setP(Constants.AutoConstants.kPYController + 0.6);
        }


        if ( m_yController.atSetpoint() && isXOutputDisabled){
             isYOutputDisabled = true;
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
        m_swerveSub.setModuleStates(moduleStates,true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("ALign_running?",false);

        m_swerveSub.stopModules();


    }

    @Override
    public boolean isFinished() {
        if(!LimelightHelpers.getTV(limelightName))
        {
            // No target found, stop.
            return true;
        }
        /*if x and y outputs are disabled because
        the robot is at its setpoint, end command. 
        robot doesnt end with its at both x and y setpoints 
        because changing the robot x position will always slightly
        change its y position, causing a steady state error.
         */
        if(
         isXOutputDisabled && isYOutputDisabled )
        {
            return true;
        }
        // You might want to add a tolerance check here to stop when close enough to the target.
        return false; // Run until interrupted by driver.
    }
}