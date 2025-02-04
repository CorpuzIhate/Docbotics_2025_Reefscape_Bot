
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.LimelightHelpers;

import com.studica.frc.AHRS;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveSub extends SubsystemBase {

    // Defines swerve modules.
    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kIsFrontRightDriveEncoderReversed,
            DriveConstants.kIsFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackRight,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kIsFrontLeftDriveEncoderReversed,
            DriveConstants.kIsFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.DriveAbsoluteEncoderOffsetRad.kFrontLeft,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kIsBackRightDriveEncoderReversed,
            DriveConstants.kIsBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackRight,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kIsBackLeftDriveEncoderReversed,
            DriveConstants.kIsBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.DriveAbsoluteEncoderOffsetRad.kBackLeft,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    /** Swerve modules states, used in debugging in AdvantageScope. */
    private final SwerveModuleState[] mySwerveStates = new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };

    /** Array of all swerve modules. */
    private final SwerveModule swerveModules[] = new SwerveModule[] {
            frontLeft, frontRight,
            backLeft, backRight
    };
    /**
     * Used in Auto targeting so that when robot loses sight of April tag
     * temporarily
     * it will have a tx value to base its auto targeting controller on.
     */
    private double initial_limeLightTX = 0;

    /** Used to get the pose of the robot. */
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositionsAuto());

    private RobotConfig config;

    /** gyro. plugged into USB port on RIO. */
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kUSB1);

    public SwerveSub() {

        new Thread(() -> { /// Try catch function is a fancy if else statement,
            try { // it tries to run a thread of resetting the gyro but if it exception e happens
                  // it stops.
                Thread.sleep(1000);
            } catch (Exception e) {
            }
        }).start();

        zeroHeading();

        /*
         * Load the RobotConfig from the GUI settings. You should probably
         * store this in your Constants file.
         */

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed.
            e.printStackTrace();
        }

        /*
         * configures the auto builder which is used in Pathplanner
         * to generate autonomous robot sequences.
         */
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier.
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose).
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE.
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds.
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class.
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants.
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants.

                ),
                config,
                () -> {
                    /*
                     * Boolean supplier that controls when the path will be mirrored for the red
                     * alliance,
                     * This will flip the path being followed to the red side of the field.
                     * THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
                     */

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    @Override
    public void periodic() {
        /* Update robot's odometer. */
        odometer.update(getRotation2d(), getModulePositionsAuto());
        /* Sends telemetry on robot heading and location. */
        SmartDashboard.putNumber("robot Heading", getHeading());
        SmartDashboard.putString("robot location", getPose().getTranslation().toString());

        /* Update swerve module positions. */
        SwerveModulePosition[] debugModulePosition = getModulePositionsAuto();
        for (int i = 0; i <= 3; ++i) {
            SmartDashboard.putString("SwerveModulePositions [" + i + "]",
                    "distance : " + debugModulePosition[i].distanceMeters
                            + "Speeds : " + debugModulePosition[i].angle);
            /* Sends robot pose telemetry to Advantagescope */
            Logger.recordOutput("pose2d", getPose());
            /* Sends swerve module positions telemetry to SmartDashBoard. */
            SmartDashboard.putNumber("SwerveModuleTurningPositions [" + 1 + "]", frontLeft.getTurningPosition());
            SmartDashboard.putNumber("SwerveModuleTurningPositions [" + 2 + "]", frontRight.getTurningPosition());
            SmartDashboard.putNumber("SwerveModuleTurningPositions [" + 3 + "]", backLeft.getTurningPosition());
            SmartDashboard.putNumber("SwerveModuleTurningPositions [" + 4 + "]", backRight.getTurningPosition());
        }

        Logger.recordOutput("heading", getHeading());

        /* Send telemetry on individual modules to SmartDashboard. */
        frontLeft.sendToDashboard();
        frontRight.sendToDashboard();
        backLeft.sendToDashboard();
        backRight.sendToDashboard();
    }

    /**
     * @return the Pose2d of the robot (translation 2d(m), rotation 2d (rad)).
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Resets the current pose(rotation2d (rad), ) of the robot.
     */
    public void resetPose(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), getModulePositionsAuto(), pose);
    }

    /**
     * Converts the module states into chassis speeds which are used for
     * calculating the inverse kinematics of the swerve modules so we can drive
     * the modules to our desired position.
     * After the conversion, it returns the chassis speeds.
     * 
     * @return speed of the chassis (x speed, y speed, turning speed) all relative
     *         to the robot.
     */
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Converts the current reference frame of chassis speeds into robot relative.
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /**
     * Sets the swerve module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // proportaionally decreases the change the speeds so driver always had control
        // of robot
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);

        // Outputs telemetry to Adavantage Log.

        // Logs desired states is an array that orders the desired states in the order
        // Advantage Log wants ( FL,FR, BL, BR ).
        SwerveModuleState[] LogDesiredStates = new SwerveModuleState[] { desiredStates[1], desiredStates[0],
                desiredStates[3], desiredStates[2] };

        Logger.recordOutput("CurrentStates", mySwerveStates);
        Logger.recordOutput("DesiredStates", LogDesiredStates);

    }

    /**
     * Gets the current positions of the all the swerve modules.
     * 
     * @return an array of all SwerveModulePosition(m).
     */
    public SwerveModulePosition[] getModulePositionsAuto() { // not updating
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getSwerveModulePosition();
        }
        return positions;
    }

    // *Zeros the gyros heading. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Converts the the gyros heading output between 0 and 360 degrees
     * because gryo is naturally continous.
     */
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    /**
     * Converts into gryos heading in radians so the value
     * can be converted into Rotation2d .
     * 
     * @return the gyro heading in rotation 2d (rad)
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * @return an array of all swerve module states
     *         (module speed in m/s , turing pos rad).
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Stops all swerve modules.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();

    }

    /**
     * Orient the robot so that the camera is facing the target directly.
     *
     */
    public double orientToTarget() {
        if (LimelightHelpers.getTV("limelight")) {
            initial_limeLightTX = LimelightHelpers.getTX("limelight");
        }
        double targetingAngularVelocity = initial_limeLightTX *
                Constants.DriveConstants.autoTargetConstants.autoOrientKp;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        // if there were apply a really small power output to the
        // turning motor, stop applying power
        if (Math.abs(targetingAngularVelocity) <= 0.001) {
            targetingAngularVelocity = 0;
        }

        return targetingAngularVelocity;

    }

}