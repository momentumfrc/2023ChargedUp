package frc.robot.subsystems;

import java.io.File;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;

import com.momentum4999.utils.PIDTuner;

public class DriveSubsystem extends SubsystemBase {
    // (6 inch diameter * (1 meters / 39.3701 inches) * PI)
    private static final double METERS_PER_REV = 6 * (1 / 39.3701) * Math.PI;
    // 1000 ticks per revolution
    private static final double ENCODER_SCALE =  METERS_PER_REV / 1000;
    private final Victor frontLeftMtr = new Victor(2);
    private final Victor rearLeftMtr = new Victor(3);
    private final Victor frontRightMtr = new Victor(0);
    private final Victor rearRightMtr = new Victor(1);

    private final Encoder frontLeftEncoder = new Encoder(0, 1);
    private final Encoder frontRightEncoder = new Encoder(2, 3);
    private final Encoder rearLeftEncoder = new Encoder(4, 5);
    private final Encoder rearRightEncoder = new Encoder(6, 7);

    private final MoPIDF frontLeftPID = new MoPIDF();
    private final MoPIDF frontRightPID = new MoPIDF();
    private final MoPIDF rearLeftPID = new MoPIDF();
    private final MoPIDF rearRightPID = new MoPIDF();

    private final PIDTuner frontLeftTuner;
    private final PIDTuner frontRightTuner;
    private final PIDTuner rearLeftTuner;
    private final PIDTuner rearRightTuner;

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner;

    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private Rotation2d initialHeading = gyro.getRotation2d();
    private Rotation2d maintainHeading = gyro.getRotation2d();

    private GenericSubscriber shouldDriveFieldOriented = MoShuffleboard.getInstance().settingsTab
        .add("Field-Oriented Drive", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private ComplexWidget doResetOrientation = MoShuffleboard.getInstance().settingsTab
        .add("Reset Drive Orientation", new InstantCommand(() -> { initialHeading = gyro.getRotation2d(); }));

    private GenericSubscriber shouldDrivePID = MoShuffleboard.getInstance().settingsTab
        .add("PID Drive", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private GenericSubscriber shouldHeadingPID = MoShuffleboard.getInstance().settingsTab
        .add("Keep Heading", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    public DriveSubsystem() {
        PIDTuner.PIDTunerSettings settings = new PIDTuner.PIDTunerSettings();
        if(RobotBase.isReal()) {
            settings.saveValuesLocation = new File("/home/lvuser/pid_constants.ini");

        frontLeftTuner = new PIDTuner("Drive FL", frontLeftPID, settings);
        frontRightTuner = new PIDTuner("Drive FR", frontRightPID, settings);
        rearLeftTuner = new PIDTuner("Drive BL", rearLeftPID, settings);
        rearRightTuner = new PIDTuner("Drive BR", rearRightPID, settings);

        frontLeftEncoder.setDistancePerPulse(ENCODER_SCALE);
        frontRightEncoder.setDistancePerPulse(ENCODER_SCALE);
        rearLeftEncoder.setDistancePerPulse(ENCODER_SCALE);
        rearRightEncoder.setDistancePerPulse(ENCODER_SCALE);

        frontLeftMtr.setInverted(true);
        rearLeftMtr.setInverted(true);

        frontRightEncoder.setReverseDirection(true);
        frontLeftEncoder.setReverseDirection(true);
        rearLeftEncoder.setReverseDirection(true);
    }

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        // TODO: get the Rotation2d from the odometry, not the gyro (so that it uses the AprilTags)
        //       Example: this.odometry.getPoseMeters().getRotation();
        Rotation2d currentHeading = gyro.getRotation2d();
        Rotation2d fieldOrientedDriveAngle;
        if(shouldDriveFieldOriented.getBoolean(true)) {
            fieldOrientedDriveAngle = currentHeading.minus(initialHeading);
        } else {
            fieldOrientedDriveAngle = new Rotation2d();
        }

        if(turnRequest != 0) {
            maintainHeading = currentHeading;
        } else if(shouldHeadingPID.getBoolean(true)) {
            turnRequest = headingController.calculate(currentHeading.getRadians(), maintainHeading.getRadians());
        }

        var wheelSpeeds = MecanumDrive.driveCartesianIK(fwdRequest, leftRequest, turnRequest, fieldOrientedDriveAngle);
        double maxSpeedRpm = MoPrefs.maxDriveRpm.get();

        if(shouldDrivePID.getBoolean(true)) {
            frontLeftMtr.set(frontLeftPID.calculate((frontLeftEncoder.getRate() / METERS_PER_REV) * 60, wheelSpeeds.frontLeft * maxSpeedRpm));
            frontRightMtr.set(frontRightPID.calculate((frontLeftEncoder.getRate() / METERS_PER_REV) * 60, wheelSpeeds.frontRight * maxSpeedRpm));
            rearLeftMtr.set(rearLeftPID.calculate((rearLeftEncoder.getRate() / METERS_PER_REV) * 60, wheelSpeeds.rearLeft * maxSpeedRpm));
            rearRightMtr.set(rearRightPID.calculate((rearRightEncoder.getRate() / METERS_PER_REV) * 60, wheelSpeeds.rearRight * maxSpeedRpm));
        } else {
            frontLeftMtr.set(wheelSpeeds.frontLeft);
            frontRightMtr.set(wheelSpeeds.frontRight);
            rearLeftMtr.set(wheelSpeeds.rearLeft);
            rearRightMtr.set(wheelSpeeds.rearRight);
        }
    }
}
