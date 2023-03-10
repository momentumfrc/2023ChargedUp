package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.utils.PIDTuner;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.PathFollowingUtils;
import frc.robot.utils.TunerUtils;

public class DriveSubsystem extends SubsystemBase {
    /**
     * How many revolutions of a wheel encoder correspond to one meter of forward travel.
     */
    public static final double REVOLUTIONS_PER_METER = 22.2; // (0.152 * Math.PI) * (1.0 / 10.71);

    /**
     * The maximum rate of turn that the drive will consider as equivalent to zero. Used to
     * determine when to re-enable heading pid after executing a driver-requested turn.
     */
    private static final double TURN_RATE_CUTOFF = 0.001;

    /**
     * The maximum rate of movement that the drive will consider as equivalent to zero.
     */
    private static final double MOVE_RATE_CUTOFF = 0.05;

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

            tuner = TunerUtils.forVenom(motor, "Drive " + mnemonic, true);
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

    public final MoPIDF xPathController = new MoPIDF();
    public final MoPIDF yPathController = new MoPIDF();
    public final MoPIDF rotPathController = new MoPIDF();

    private final PIDTuner xPathTuner = TunerUtils.forMoPID(xPathController, "X Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);
    private final PIDTuner yPathTuner = TunerUtils.forMoPID(yPathController, "Y Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);
    private final PIDTuner rotPathTuner = TunerUtils.forMoPID(rotPathController, "Rot Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner = TunerUtils.forMoPIDF(headingController, "Drive Heading", true);

    private final AHRS gyro;

    private Rotation2d maintainHeading;

    private GenericSubscriber shouldDrivePID = MoShuffleboard.getInstance().settingsTab
        .add("PID Drive", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private GenericSubscriber shouldHeadingPID = MoShuffleboard.getInstance().settingsTab
        .add("Keep Heading", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;
        maintainHeading = gyro.getRotation2d();

        frontLeftMtr.motor.setInverted(true);
        rearLeftMtr.motor.setInverted(true);
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
            -1 * frontLeftMtr.motor.getPosition() / REVOLUTIONS_PER_METER,
            -1 * frontRightMtr.motor.getPosition() / REVOLUTIONS_PER_METER,
            -1 * rearLeftMtr.motor.getPosition() / REVOLUTIONS_PER_METER,
            -1 * rearRightMtr.motor.getPosition() / REVOLUTIONS_PER_METER
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
        this.driveCartesian(fwdRequest, leftRequest, turnRequest, new Rotation2d());
    }

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        var wheelSpeeds = MecanumDrive.driveCartesianIK(fwdRequest, leftRequest, calculateTurn(turnRequest, gyro.getRotation2d()), fieldOrientedDriveAngle.unaryMinus());
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

    public void stop() {
        frontLeftMtr.motor.setBrakeCoastMode(BrakeCoastMode.Brake);
        frontRightMtr.motor.setBrakeCoastMode(BrakeCoastMode.Brake);
        rearLeftMtr.motor.setBrakeCoastMode(BrakeCoastMode.Brake);
        rearRightMtr.motor.setBrakeCoastMode(BrakeCoastMode.Brake);

        frontLeftMtr.motor.setCommand(ControlMode.Proportional, 0);
        frontRightMtr.motor.setCommand(ControlMode.Proportional, 0);
        rearLeftMtr.motor.setCommand(ControlMode.Proportional, 0);
        rearRightMtr.motor.setCommand(ControlMode.Proportional, 0);
    }

    public void driveWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
        if(!shouldDrivePID.getBoolean(true)) {
            DriverStation.reportWarning("Cannot driveWheelSpeeds() with PID disabled", false);
            stop();
            return;
        }

        frontLeftMtr.motor.setCommand(ControlMode.SpeedControl, -1 * speeds.frontLeftMetersPerSecond * REVOLUTIONS_PER_METER);
        frontRightMtr.motor.setCommand(ControlMode.SpeedControl, -1 * speeds.frontRightMetersPerSecond * REVOLUTIONS_PER_METER);
        rearLeftMtr.motor.setCommand(ControlMode.SpeedControl, -1 * speeds.rearLeftMetersPerSecond * REVOLUTIONS_PER_METER);
        rearRightMtr.motor.setCommand(ControlMode.SpeedControl, -1 * speeds.rearRightMetersPerSecond * REVOLUTIONS_PER_METER);
    }

    public void driveDifferentialWheelSpeeds(double leftMps, double rightMps) {
        if(!shouldDrivePID.getBoolean(true)) {
            DriverStation.reportWarning("Cannot driveWheelSpeeds() with PID disabled", false);
            stop();
            return;
        }

        frontLeftMtr.motor.setCommand(ControlMode.SpeedControl, -1 * leftMps * 60 * REVOLUTIONS_PER_METER);
        frontRightMtr.motor.setCommand(ControlMode.SpeedControl, -1 * rightMps * 60 * REVOLUTIONS_PER_METER);
        rearLeftMtr.motor.setCommand(ControlMode.SpeedControl, -1 * leftMps * 60 * REVOLUTIONS_PER_METER);
        rearRightMtr.motor.setCommand(ControlMode.SpeedControl, -1 * rightMps * 60 * REVOLUTIONS_PER_METER);

    }

    public boolean isMoving() {
        return Math.abs(gyro.getRate()) > TURN_RATE_CUTOFF
            || Math.abs(frontLeftMtr.motor.getSpeed()) > MOVE_RATE_CUTOFF
            || Math.abs(frontRightMtr.motor.getSpeed()) > MOVE_RATE_CUTOFF
            || Math.abs(rearLeftMtr.motor.getSpeed()) > MOVE_RATE_CUTOFF
            || Math.abs(rearRightMtr.motor.getSpeed()) > MOVE_RATE_CUTOFF;
    }

    public void resetMaintainHeading() {
        turnState = TurnState.TURNING;
    }
}
