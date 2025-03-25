
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

public class AlignToBranchCMD extends Command {
    private final SwerveSub m_swerveSub;
    private final boolean  m_alignLeft; // true for left branch, false for right
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;

    /**this is the name of Limelight we are using depending on if 
    * we're aligning to the left of right branch of the reef.
    */
    private String MainLimelight = Constants.LimeLightConstants.LeftLimeLight;
    private double targetYOffset = Constants.LimeLightConstants.kRightBranchTargetYOffset; 
 
    public AlignToBranchCMD(SwerveSub swerveSub, boolean  alignLeft) {
        m_swerveSub = swerveSub;
        m_alignLeft = alignLeft;

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
        SmartDashboard.putBoolean("m_alignLeft?",m_alignLeft);



        /**if were aligning to the left reef branch
         * use the Right Limelight and use left y Offset. 
         * otherwise use the Left 
         * LimeLight and right y offset.
         */
        if(m_alignLeft){
            MainLimelight = Constants.LimeLightConstants.RightLimeLight;
            targetYOffset = Constants.LimeLightConstants.kLeftBranchTargetYOffset; 

        }

        SmartDashboard.putBoolean("running?",true);
    }

    @Override
    public void execute() {

        
        // Adjust these target offsets based on your robot and field setup.
        SmartDashboard.putNumber("targetYOffset?",targetYOffset);
        


        double tx = LimelightHelpers.getTX(MainLimelight);
        double ty = LimelightHelpers.getTY(MainLimelight);

        /*
         * x -> forward/ back from the robot
         * y -> left / right from the robot
         * 
         */




        
        double currentHeading = m_swerveSub.getHeading();
        double desiredHeading = 60;  //FIX ME

        double xDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace(MainLimelight).getZ();
        double yDistanceFromTarget_meters = LimelightHelpers.getBotPose3d_TargetSpace(MainLimelight).getX();


        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);


        SmartDashboard.putData("limeLight_xController", m_xController);
        SmartDashboard.putData("limelight_yController", m_yController);
        SmartDashboard.putData("limelight_thetaController",m_rotationController);
        



        // Calculate PID outputs.
        double xOutput = m_xController.calculate(xDistanceFromTarget_meters, Constants.LimeLightConstants.kLeftBranchTargetYOffset); // replace 0 with current robot x.
        double yOutput = m_yController.calculate(yDistanceFromTarget_meters, targetYOffset); // replace 0 with current robot y.
        double rotationOutput = -m_rotationController.calculate(currentHeading, desiredHeading); 
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
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, -yOutput, -rotationOutput);
        /*convert chassis speeds into module states */
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putNumber("rotationOutput", rotationOutput);
        SmartDashboard.putNumber("xOutput", xOutput);
        SmartDashboard.putNumber("yOutput", yOutput);


        
        // field relative
        m_swerveSub.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("running?",false);

        m_swerveSub.stopModules();

    }

    @Override
    public boolean isFinished() {
        if(!LimelightHelpers.getTV(MainLimelight))
        {
            // No target found, stop.
            m_swerveSub.stopModules();
            return true;
        }
        // You might want to add a tolerance check here to stop when close enough to the target.
        return false; // Run until interrupted by driver.
    }
}
