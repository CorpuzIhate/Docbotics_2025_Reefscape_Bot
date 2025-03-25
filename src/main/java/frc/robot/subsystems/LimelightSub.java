package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;

public class LimelightSub extends SubsystemBase {
    
    /**Check if robot is in the correct y Pos for branch Alignment */
    public boolean isYPosOrientedToTarget(double robotYPos_meter) {
        return 
            robotYPos_meter > LimeLightConstants.kBranchTargetYOffset + LimeLightConstants.kBranchTargetYOffsetTolerance 
            &&
            robotYPos_meter < LimeLightConstants.kBranchTargetYOffset - LimeLightConstants.kBranchTargetYOffsetTolerance ;
    }
    /**Check if robot is in the correct x Pos for branch Alignment */

    public boolean isXPosOrientedToTarget(double robotXPos_meter) {
        return robotXPos_meter <= LimeLightConstants.kLeftBranchTargetYOffset;
    }
}
