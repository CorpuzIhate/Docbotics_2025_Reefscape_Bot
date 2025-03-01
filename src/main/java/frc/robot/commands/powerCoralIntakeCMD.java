package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;

public class powerCoralIntakeCMD extends Command {
    private final CoralIntakeConsumerSub intakeConsumerSub;
    private final SparkMax intakeConsumerMotor;
    public final Supplier<Double> intakeConsumerSpeedSupplier;
    public final Supplier<Double> outakeConsumerSpeedSupplier;


    public powerCoralIntakeCMD(
        CoralIntakeConsumerSub intakeConsumerSub, 
    Supplier<Double> intakeConsumerSpeedSupplier,
    Supplier<Double> outakeConsumerSpeedSupplier) {
        this.intakeConsumerSub = intakeConsumerSub;
        this.intakeConsumerMotor = intakeConsumerSub.getIntakeConsumerMotor();
        this.intakeConsumerSpeedSupplier = intakeConsumerSpeedSupplier;
        this.outakeConsumerSpeedSupplier = outakeConsumerSpeedSupplier;
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
        intakeConsumerMotor.set( 
            (intakeConsumerSpeedSupplier.get() * 0.3) - 
        (outakeConsumerSpeedSupplier.get() * 0.3) );
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();
        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);

    }

    @Override
    public boolean isFinished() {

        return false;
    }
}