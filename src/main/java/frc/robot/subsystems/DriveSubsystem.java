package frc.robot.subsystems;

import java.io.File;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.VenomTunerAdapter;

import com.momentum4999.utils.PIDTuner;

public class DriveSubsystem extends SubsystemBase {
    private final CANVenom frontLeftMtr = new CANVenom(Constants.DRIVE_LEFT_FRONT.address);
    private final CANVenom frontRightMtr = new CANVenom(Constants.DRIVE_RIGHT_FRONT.address);
    private final CANVenom rearLeftMtr = new CANVenom(Constants.DRIVE_LEFT_REAR.address);
    private final CANVenom rearRightMtr = new CANVenom(Constants.DRIVE_RIGHT_REAR.address);

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
        }

        frontLeftTuner = new PIDTuner("Drive FL", new VenomTunerAdapter(frontLeftMtr), settings);
        frontRightTuner = new PIDTuner("Drive FR", new VenomTunerAdapter(frontRightMtr), settings);
        rearLeftTuner = new PIDTuner("Drive BL", new VenomTunerAdapter(rearLeftMtr), settings);
        rearRightTuner = new PIDTuner("Drive BR", new VenomTunerAdapter(rearRightMtr), settings);
        headingTuner = new PIDTuner("Drive Heading", headingController, settings);
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
            frontLeftMtr.setCommand(ControlMode.SpeedControl, wheelSpeeds.frontLeft * maxSpeedRpm);
            frontRightMtr.setCommand(ControlMode.SpeedControl, wheelSpeeds.frontRight * maxSpeedRpm);
            rearLeftMtr.setCommand(ControlMode.SpeedControl, wheelSpeeds.rearLeft * maxSpeedRpm);
            rearRightMtr.setCommand(ControlMode.SpeedControl, wheelSpeeds.rearRight * maxSpeedRpm);
        } else {
            frontLeftMtr.setCommand(ControlMode.Proportional, wheelSpeeds.frontLeft);
            frontRightMtr.setCommand(ControlMode.Proportional, wheelSpeeds.frontRight);
            rearLeftMtr.setCommand(ControlMode.Proportional, wheelSpeeds.rearLeft);
            rearRightMtr.setCommand(ControlMode.Proportional, wheelSpeeds.rearRight);
        }
    }
}
