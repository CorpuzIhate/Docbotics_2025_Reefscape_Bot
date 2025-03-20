
package frc.robot.commands;

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
    private final boolean m_alignLeft; // true for left branch, false for right
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
 
    public AlignToBranchCMD(SwerveSub swerveSub, boolean alignLeft) {
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

        SmartDashboard.putBoolean("running?",true);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("running?",true);

        if(!LimelightHelpers.getTV(""))
        {
            // No target found, stop.
            m_swerveSub.stopModules();
            return;
        }

        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");

        /*
         * x -> forward/ back from the robot
         * y -> left / right from the robot
         * 
         */

        // Adjust these target offsets based on your robot and field setup.
        double targetXOffset = m_alignLeft ? Constants.AutoConstants.kLeftBranchTargetXOffset : Constants.AutoConstants.kRightBranchTargetXOffset;
        double targetYOffset = Constants.AutoConstants.kBranchTargetYOffset; // common Y offset
        double targetRotationOffset = m_alignLeft ? Constants.AutoConstants.kLeftBranchTargetRotationOffset : Constants.AutoConstants.kRightBranchTargetRotationOffset;
        
        double currentHeading = m_swerveSub.getHeading();
        double desiredHeading = 0; //FIX ME

        /*FOR NOW, NO OFFSETS */
        double distanceFromRobotToTarget_inches = 
        (Constants.LimelightConstants.limelightHeight_inches - Constants.LimelightConstants.reefTargetHeight_Inches) /
         Math.tan(Math.toRadians(Constants.LimelightConstants.kLimeLightAngleFromGround_degrees - ty));
        
        double targetXRelativeField = distanceFromRobotToTarget_inches * Math.sin(Math.toRadians(desiredHeading - currentHeading + tx));
        double targetYRelativeField = distanceFromRobotToTarget_inches * Math.cos(Math.toRadians(desiredHeading - currentHeading + tx));


        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putNumber("desiredHeading", desiredHeading);





        // Calculate desired robot position based on limelight data and target offsets.
        double desiredX = targetXOffset; // Replace with actual calculations based on limelight and desired position.
        double desiredY = targetYOffset; // Replace with actual calculations based on limelight and desired position.
        double desiredRotation = targetRotationOffset; // Replace with actual calculations based on limelight and desired rotation.
        
        SmartDashboard.putData("limeLight_xController", m_xController);
        SmartDashboard.putData("limelight_yController", m_yController);


        SmartDashboard.putData("limelight_thetaController",m_rotationController);
        // Calculate PID outputs.
        double xOutput = m_xController.calculate(targetYRelativeField, 0); // replace 0 with current robot x.
        double yOutput = -m_yController.calculate(targetXRelativeField, 0); // replace 0 with current robot y.
        double rotationOutput = -m_rotationController.calculate(currentHeading, desiredHeading); 
        if (Math.abs(rotationOutput) < 0.05){
            rotationOutput = 0;
        }
        if (Math.abs(xOutput) < 0.05){
            xOutput = 0;
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
        // You might want to add a tolerance check here to stop when close enough to the target.
        return false; // Run until interrupted by driver.
    }
}
