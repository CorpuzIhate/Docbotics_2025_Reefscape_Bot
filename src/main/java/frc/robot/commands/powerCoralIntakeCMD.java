package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;


public class powerCoralIntakeCMD extends Command {

    private final SparkMax intakeConsumerMotor;
    public final Supplier<Double> intakeConsumerSpeedSupplier;
    public final Supplier<Double> outakeConsumerSpeedSupplier;


    public powerCoralIntakeCMD(
        CoralIntakeConsumerSub intakeConsumerSub, 
    Supplier<Double> intakeConsumerSpeedSupplier,
    Supplier<Double> outakeConsumerSpeedSupplier) {
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
       
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}