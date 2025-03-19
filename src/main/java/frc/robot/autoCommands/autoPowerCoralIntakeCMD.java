package frc.robot.autoCommands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;

public class autoPowerCoralIntakeCMD extends Command {
    private final CoralIntakeConsumerSub intakeConsumerSub;
    private final SparkMax intakeConsumerMotor;
    public final double power;



    public autoPowerCoralIntakeCMD(
        CoralIntakeConsumerSub intakeConsumerSub, 
        double power ) {
        this.intakeConsumerSub = intakeConsumerSub;
        this.intakeConsumerMotor = intakeConsumerSub.getIntakeConsumerMotor();
        this.power = power;
        addRequirements(intakeConsumerSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        /* power intake consumer motor */
        intakeConsumerMotor.set(power);
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}