package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSub;

public class ResetHeadingCMD  extends Command  {
    private final SwerveSub swerveSub;

    public ResetHeadingCMD(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
         
    }
    @Override
    public void execute() {
        //Reset the the gyro of the robot.
        swerveSub.zeroHeading();
    }


}
