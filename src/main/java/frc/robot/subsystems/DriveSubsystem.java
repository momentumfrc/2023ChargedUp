package frc.robot.subsystems;

import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.VenomTunerAdapter;

import com.momentum4999.utils.PIDTuner;

public class DriveSubsystem extends SubsystemBase implements Consumer<Pose3d> {
    /**
     * The maximum rate of turn that the drive will consider as equivalent to zero. Used to
     * determine when to re-enable heading pid after executing a driver-requested turn.
     */
    private static final double TURN_RATE_CUTOFF = 0.001;

    /**
     * How many meters of forward travel correspond to 1 revolution of a wheel encoder.
     */
    private static final double METERS_PER_REVOLUTION = (6 * Math.PI) * (1.0 / 10.71);

    /**
     * The distance, in meters, of a wheel from the center of the robot towards the
     * front of the robot.
     */
    private static final double WHEEL_FWD_POS = 0.2664394;

    /**
     * The distance, in meters, of a wheel from the center of the robot towards the
     * left side of the robot.
     */
    private static final double WHEEL_LEFT_POS = 0.294386;

    private static enum TurnState {
        TURNING,
        HOLD_HEADING
    };
    private TurnState turnState = TurnState.HOLD_HEADING;

    private static class DriveMotor {
        private final String mnemonic;
        private final PIDTuner tuner;
        public final CANVenom motor;
        public DriveMotor(String mnemonic, Constants.CANAddress address) {
            this.mnemonic = mnemonic;
            motor = new CANVenom(address.address);
            motor.setControlMode(ControlMode.SpeedControl);
            motor.setBrakeCoastMode(BrakeCoastMode.Brake);
            motor.setPID(0, 0, 0, 0, 0);
            motor.setMaxJerk(0);

            tuner = new PIDTuner("Drive " + mnemonic, new VenomTunerAdapter(motor), Constants.TUNER_SETTINGS);
        }

        @Override
        public String toString() {
            return String.format("DriveMotor(\"%s\")", this.mnemonic);
        }
    }

    private final DriveMotor frontLeftMtr = new DriveMotor("FL", Constants.DRIVE_LEFT_FRONT);
    private final DriveMotor frontRightMtr = new DriveMotor("FR", Constants.DRIVE_RIGHT_FRONT);
    private final DriveMotor rearLeftMtr = new DriveMotor("BL", Constants.DRIVE_LEFT_REAR);
    private final DriveMotor rearRightMtr = new DriveMotor("BR", Constants.DRIVE_RIGHT_REAR);

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner;

    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private Rotation2d initialHeading = gyro.getRotation2d();
    private Rotation2d maintainHeading = gyro.getRotation2d();

    private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(WHEEL_FWD_POS, WHEEL_LEFT_POS),
        new Translation2d(WHEEL_FWD_POS, -WHEEL_LEFT_POS),
        new Translation2d(-WHEEL_FWD_POS, WHEEL_LEFT_POS),
        new Translation2d(-WHEEL_FWD_POS, -WHEEL_LEFT_POS)
    );
    private MecanumDriveOdometry odometry = new MecanumDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        getWheelPositions()
    );
    private Pose2d currPose = odometry.getPoseMeters();

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

    private Field2d field = MoShuffleboard.getInstance().field;

    public DriveSubsystem() {
        headingTuner = new PIDTuner("Drive Heading", headingController, Constants.TUNER_SETTINGS);

        frontLeftMtr.motor.setInverted(true);
        rearLeftMtr.motor.setInverted(true);
    }

    private MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
            frontLeftMtr.motor.getPosition() * METERS_PER_REVOLUTION,
            frontRightMtr.motor.getPosition() * METERS_PER_REVOLUTION,
            rearLeftMtr.motor.getPosition() * METERS_PER_REVOLUTION,
            rearRightMtr.motor.getPosition() * METERS_PER_REVOLUTION
        );
    }

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
        Rotation2d currentHeading = currPose.getRotation();
        Rotation2d fieldOrientedDriveAngle;
        if(shouldDriveFieldOriented.getBoolean(true)) {
            fieldOrientedDriveAngle = currentHeading.minus(initialHeading);
        } else {
            fieldOrientedDriveAngle = new Rotation2d();
        }

        var wheelSpeeds = MecanumDrive.driveCartesianIK(fwdRequest, leftRequest, calculateTurn(turnRequest, currentHeading), fieldOrientedDriveAngle.unaryMinus());
        double maxSpeedRpm = MoPrefs.maxDriveRpm.get();

        if(shouldDrivePID.getBoolean(true)) {
            frontLeftMtr.motor.setCommand(ControlMode.SpeedControl, wheelSpeeds.frontLeft * maxSpeedRpm);
            frontRightMtr.motor.setCommand(ControlMode.SpeedControl, wheelSpeeds.frontRight * maxSpeedRpm);
            rearLeftMtr.motor.setCommand(ControlMode.SpeedControl, wheelSpeeds.rearLeft * maxSpeedRpm);
            rearRightMtr.motor.setCommand(ControlMode.SpeedControl, wheelSpeeds.rearRight * maxSpeedRpm);
        } else {
            frontLeftMtr.motor.setCommand(ControlMode.Proportional, wheelSpeeds.frontLeft);
            frontRightMtr.motor.setCommand(ControlMode.Proportional, wheelSpeeds.frontRight);
            rearLeftMtr.motor.setCommand(ControlMode.Proportional, wheelSpeeds.rearLeft);
            rearRightMtr.motor.setCommand(ControlMode.Proportional, wheelSpeeds.rearRight);
        }
    }

    public void resetMaintainHeading() {
        turnState = TurnState.TURNING;
    }

    @Override
    public void periodic() {
        currPose = odometry.update(gyro.getRotation2d(), getWheelPositions());
        field.setRobotPose(currPose);
    }

    @Override
    public void accept(Pose3d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose.toPose2d());
    }
}
