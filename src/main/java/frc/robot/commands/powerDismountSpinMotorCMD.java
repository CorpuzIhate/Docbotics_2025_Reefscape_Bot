package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;
import frc.robot.subsystems.DismountSub;

public class powerDismountSpinMotorCMD extends Command {
    private final DismountSub dismountSub;
    private final SparkMax dismountSpinMotor;
    public final double power;



    public powerDismountSpinMotorCMD(  DismountSub dismountSub, 
        double power ) {
        this.dismountSub = dismountSub;
        this.dismountSpinMotor = dismountSub.getDismountSpinMotor();
        this.power = power;
        addRequirements(dismountSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake consumer motor. */
        dismountSpinMotor.set(0);
        dismountSpinMotor.stopMotor();

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        /* power intake consumer motor */
        dismountSpinMotor.set(power);
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake consumer motor. */
        dismountSpinMotor.set(0);
        dismountSpinMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}