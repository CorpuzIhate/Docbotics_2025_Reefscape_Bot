
package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.LimelightSub;
public class ManageLimeLightCMD extends Command{

    private final LimelightSub limelightSub;
    public ManageLimeLightCMD(LimelightSub limelightSub){
        this.limelightSub = limelightSub;
        addRequirements(limelightSub);
    }
    @Override
    public void execute(){
        //Telemetry.
        SmartDashboard.putBoolean("isTarget",LimelightHelpers.getTV("limelight") );
  
    }
}
