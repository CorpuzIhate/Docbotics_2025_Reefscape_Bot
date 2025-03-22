
package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.Vector;
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

public class AlignToBranchCMD extends Command {
    private final SwerveSub m_swerveSub;
    private final Supplier<Boolean>  m_alignLeft; // true for left branch, false for right
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    private final Translation2d m_initalRobotPos_meters;



 
    public AlignToBranchCMD(SwerveSub swerveSub, Supplier<Boolean>  alignLeft) {
        m_swerveSub = swerveSub;
        m_alignLeft = alignLeft;

        m_xController = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController);
        m_yController = new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController);
        m_rotationController = new PIDController(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, Constants.AutoConstants.kDThetaController);

        m_rotationController.enableContinuousInput(-180, 180); // important for heading

        /*gets the initial displacement from the  from where the robot started
        of the robot in meters.
         */
        m_initalRobotPos_meters = swerveSub.getPose().getTranslation();


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

        // Adjust these target offsets based on your robot and field setup.
        double targetYOffset = Constants.AutoConstants.kBranchTargetYOffset; // common Y offset
        if(m_alignLeft.get()){
            targetYOffset *= -1;
        }

        SmartDashboard.putBoolean("m_alignLeft?",m_alignLeft.get());

        SmartDashboard.putNumber("targetYOffset?",targetYOffset);

        
        
        /**gets the current heading of the robot in degrees. The is from -180 to 180 */
        double currentHeading = m_swerveSub.getHeading();
        /**sets the desired gyro heading */

        double desiredHeading = 0; //FIX ME
        

        /*gets the current displacement from where the robot started
        of the robot in meters.
         */
        Translation2d currentRobotPos_meters = m_swerveSub.getPose().getTranslation();

        /*gets displacement from initial position from when the 
         * command starts.
          */
        Translation2d displacementFromInitialPos = 
        currentRobotPos_meters.minus(m_initalRobotPos_meters);    

        /**Sends telemetry related to the alignment with the reef branch. */
        SmartDashboard.putNumber("displacementFromInitialPos.getX()", displacementFromInitialPos.getX());
        SmartDashboard.putNumber("displacementFromInitialPos.getY()", displacementFromInitialPos.getY());

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        SmartDashboard.putData("limeLight_xController", m_xController);
        SmartDashboard.putData("limelight_yController", m_yController);
        SmartDashboard.putData("limelight_thetaController",m_rotationController);

        // Calculate PID outputs.
        double xOutput = m_xController.calculate(displacementFromInitialPos.getX(), 1.1); 
        double yOutput = m_yController.calculate(displacementFromInitialPos.getY(), targetYOffset); 
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
            m_swerveSub.stopModules();
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
