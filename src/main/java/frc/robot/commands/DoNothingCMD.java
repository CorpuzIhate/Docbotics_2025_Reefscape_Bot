package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;


public class DoNothingCMD extends SequentialCommandGroup {
    
public DoNothingCMD(){ 
    SmartDashboard.putBoolean( "Do Nothing?", true);

}

}