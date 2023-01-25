package frc.robot.subsystems;

import java.io.File;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private static final double TURN_RATE_CUTOFF = 0.001;

    private static enum TurnState {
        TURNING,
        HOLD_HEADING
    };
    private TurnState turnState = TurnState.HOLD_HEADING;

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

        frontLeftMtr.setControlMode(ControlMode.SpeedControl);
        frontRightMtr.setControlMode(ControlMode.SpeedControl);
        rearLeftMtr.setControlMode(ControlMode.SpeedControl);
        rearRightMtr.setControlMode(ControlMode.SpeedControl);

        frontLeftMtr.setBrakeCoastMode(BrakeCoastMode.Brake);
        frontRightMtr.setBrakeCoastMode(BrakeCoastMode.Brake);
        rearLeftMtr.setBrakeCoastMode(BrakeCoastMode.Brake);
        rearRightMtr.setBrakeCoastMode(BrakeCoastMode.Brake);

        frontLeftMtr.setPID(0, 0, 0, 0, 0);
        frontRightMtr.setPID(0, 0, 0, 0, 0);
        rearLeftMtr.setPID(0, 0, 0, 0, 0);
        rearRightMtr.setPID(0, 0, 0, 0, 0);

        frontLeftMtr.setMaxJerk(0);
        frontRightMtr.setMaxJerk(0);
        rearLeftMtr.setMaxJerk(0);
        rearRightMtr.setMaxJerk(0);

        frontLeftTuner = new PIDTuner("Drive FL", new VenomTunerAdapter(frontLeftMtr), settings);
        frontRightTuner = new PIDTuner("Drive FR", new VenomTunerAdapter(frontRightMtr), settings);
        rearLeftTuner = new PIDTuner("Drive BL", new VenomTunerAdapter(rearLeftMtr), settings);
        rearRightTuner = new PIDTuner("Drive BR", new VenomTunerAdapter(rearRightMtr), settings);
        headingTuner = new PIDTuner("Drive Heading", headingController, settings);

        frontLeftMtr.setInverted(true);
        rearLeftMtr.setInverted(true);
    }

    GenericPublisher pub = NetworkTableInstance.getDefault().getTopic("PID Setpoint").getGenericEntry();

    /**
     * Calculate how much the robot should turn. We want to use PID to prevent any turning, except
     * for these situations: (1) if the driver has requested a turn, or (2) if the robot is
     * slowing down after a requested turn (to prevent an unexpected 'snap-back' behavior).
     * @param turnRequest The turn requested by the driver
     * @param currentHeading The robot's current heading, as reported by the gyro
     * @return How much the robot should turn
     */
    private double calculateTurn(double turnRequest, Rotation2d currentHeading) {
        switch(turnState) {
            case HOLD_HEADING:
                if(turnRequest != 0) {
                    turnState = TurnState.TURNING;
                }
                break;
            case TURNING:
                if(turnRequest == 0 && Math.abs(gyro.getRate()) < TURN_RATE_CUTOFF) {
                    maintainHeading = currentHeading;
                    turnState = TurnState.HOLD_HEADING;
                }
                break;
        }
        switch(turnState) {
            case HOLD_HEADING:
                if(shouldHeadingPID.getBoolean(true)) {
                    return headingController.calculate(currentHeading.getRadians(), maintainHeading.getRadians());
                }
            case TURNING:
            default:
                return turnRequest;
        }
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

        var wheelSpeeds = MecanumDrive.driveCartesianIK(fwdRequest, leftRequest, calculateTurn(turnRequest, currentHeading), fieldOrientedDriveAngle);
        double maxSpeedRpm = MoPrefs.maxDriveRpm.get();

        pub.setDouble(wheelSpeeds.frontLeft * maxSpeedRpm);

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
