package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

    private final SparkMax turningMotor;
    private final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
    // ** Used for turning the wheels to the desired angle. */
    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean isAbsoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(
            int driveMotorID,
            int turningMotorID,
            boolean driveMotorReversed,
            boolean turningMotorReversed,
            int absoluteEncoderID,
            double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.isAbsoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        // Configures drive and turning motor of
        // the swerve module.
        driveMotorConfig
                .inverted(driveMotorReversed);
        turnMotorConfig
                .inverted(turningMotorReversed);

        /* Sets the unit conversion factors of the swerve module encoders. */
        driveMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turnMotorConfig.encoder
                .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
                .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        /* Sets turning controllers PID Constants. */
        turningPidController = new PIDController(ModuleConstants.kTurningControllerPValue, 0, 0);

        /*
         * Sets the PID controller to a continuous input from -pi to pi radians, this is
         * useful because
         * this controller is rotating our wheels. For example, if we have an angle at
         * 3.1 radians and
         * my set-point is at 0 radians, it will turn the 0.04 radians towards the
         * set-point rather than turning
         * -3.1 radians.
         */
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        /**
         * Applies the motor configurations to the Motors.
         * Parameters will reset if they are changed.
         * No parameters are saved after a power cycle.
         */
        turningMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    /**
     * Gets the drive encoder's measured drive position of the module.
     * 
     * @return an encoder position in meters.
     */
    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    /**
     * Returns the turning encoder's measured turning motor
     * position of the module
     * 
     * @return an encoder position in radians from -pi to +pi.
     */
    public double getTurningPosition() {
        return ((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI) - absoluteEncoderOffsetRad);
    }

    /**
     * Returns the drive encoder's measured
     * drive velocity of the module.
     * 
     * @return drive velocity in meters per second.
     */
    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    /**
     * Returns the turning encoder's measured
     * velocity of the module.
     * 
     * @return turning velocity in radians per second.
     */
    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    /**
     * Gets the absolute encoder position.
     * 
     * @return modules absolute encoder position in radians.
     */
    public double getAbsoluteEncoderRad() {

        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        // Gives the how much percent of a rotation were reading.
        angle *= 2.0 * Math.PI; // Convert to radians.
        angle -= absoluteEncoderOffsetRad;

        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0); // Gives the Encoder value based on if the Encoder is
                                                                 // reversed.
    }

    /** Resets all encoders of the module. */
    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad()); // Resets the turning encoder to absolute
                                                                        // encoder value
    }

    /**
     * Gets the the current swerve module state.
     * 
     * @return swerve state which is the current velocity in m/s and
     *         rotation (in radians) of the swerve module.
     * 
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Gets the the current swerve module position.
     * 
     * @return swerve position which is the current position and rotation 2d (rad)
     * 
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Sets the desired speeds and rotation of the swerve module
     *
     * @param state
     *            current state of the module, which is velocity(m/s) and
     *            rotation(radians).
     */
    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.001) // if were barely moving do not reset the motors
        {
            stop();
            return;
        }
        /*
         * Minimize the change in heading this swerve module state would require by
         * potentially reversing
         * the direction the wheel spins. If this is used with the PIDController class's
         * continuous input
         * functionality, the furthest a wheel will ever rotate is 90 degrees.
         */
        state.optimize(state.angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());

    }

    /*
     * Stops driving and turning speeds.
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);

    }

    /*
     * Sends telemetry of the swerveModule.
     */
    public void sendToDashboard() {

        SmartDashboard.putNumber("Drive[" + absoluteEncoder.getDeviceID() + "] output", driveMotor.getAppliedOutput());

        SmartDashboard.putNumber("Turning[" + absoluteEncoder.getDeviceID() + "] output",
                turningMotor.getAppliedOutput());

        SmartDashboard.putNumber("DrivePos[" + absoluteEncoder.getDeviceID() + "]", getDrivePosition());

        SmartDashboard.putNumber("TurningPos[" + absoluteEncoder.getDeviceID() + "]", getTurningPosition());

        SmartDashboard.putNumber("AbsPos[" + absoluteEncoder.getDeviceID() + "] ",
                absoluteEncoder.getAbsolutePosition().getValueAsDouble());

    }

}