package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.motune.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.SwerveModule;
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
    private Rotation2d maintainHeading;

    private final MoPIDF headingController = new MoPIDF();
    private final PIDTuner headingTuner = TunerUtils.forMoPIDF(headingController, "Drive Heading");

    private GenericSubscriber shouldHeadingPID = MoShuffleboard.getInstance().settingsTab
        .add("Keep Heading", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule rearLeft;
    public final SwerveModule rearRight;

    public final MoPIDF xPathController = new MoPIDF();
    public final MoPIDF yPathController = new MoPIDF();
    public final MoPIDF rotPathController = new MoPIDF();

    private final PIDTuner xPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path X");
    private final PIDTuner yPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path Y");
    private final PIDTuner rotPathTuner = TunerUtils.forMoPID(xPathController, "Follow Path Rot");

    public final SwerveDriveKinematics kinematics;

    private final AHRS gyro;

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;
        maintainHeading = getCurrHeading();

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.frontLeft = new SwerveModule(
            "FL",
            new CANSparkMax(Constants.TURN_LEFT_FRONT.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_LEFT_FRONT.address),
            MoPrefs.flZero,
            MoPrefs.flScale,
            MoPrefs.flDriveMtrScale
        );

        this.frontRight = new SwerveModule(
            "FR",
            new CANSparkMax(Constants.TURN_RIGHT_FRONT.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_RIGHT_FRONT.address),
            MoPrefs.frZero,
            MoPrefs.frScale,
            MoPrefs.frDriveMtrScale
        );

        this.rearLeft = new SwerveModule(
            "RL",
            new CANSparkMax(Constants.TURN_LEFT_REAR.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_LEFT_REAR.address),
            MoPrefs.rlZero,
            MoPrefs.rlScale,
            MoPrefs.rlDriveMtrScale
        );

        this.rearRight = new SwerveModule(
            "RR",
            new CANSparkMax(Constants.TURN_RIGHT_REAR.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_RIGHT_REAR.address),
            MoPrefs.rrZero,
            MoPrefs.rrScale,
            MoPrefs.rrDriveMtrScale
        );

        MoShuffleboard.getInstance().matchTab.addDouble("FL_POS", frontLeft.driveMotor::getSelectedSensorPosition);
        MoShuffleboard.getInstance().matchTab.addDouble("FL_POS_m", () -> frontLeft.driveMotor.getSelectedSensorPosition() / MoPrefs.flDriveMtrScale.get());

        double xoff = MoPrefs.chassisSizeX.get() / 2;
        double yoff = MoPrefs.chassisSizeY.get() / 2;

        Translation2d fl = new Translation2d(xoff, yoff);
        Translation2d fr = new Translation2d(xoff, -yoff);
        Translation2d rl = new Translation2d(-xoff, yoff);
        Translation2d rr = new Translation2d(-xoff, -yoff);


        this.kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);
    }

    public SwerveModulePosition[] getWheelPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };
    }

    /**
     * Gets the current heading, within the range (-PI, PI]
     * @return the current heading
     */
    private Rotation2d getCurrHeading() {
        Rotation2d gyroHeading = gyro.getRotation2d();
        return new Rotation2d(gyroHeading.getCos(), gyroHeading.getSin());
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
        turnRequest = calculateTurn(turnRequest, getCurrHeading());

        double maxLinearSpeed = MoPrefs.maxDriveSpeed.get();
        double maxAngularSpeed = MoPrefs.maxTurnSpeed.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -leftRequest * maxLinearSpeed,
            -fwdRequest * maxLinearSpeed,
            -turnRequest * maxAngularSpeed,
            fieldOrientedDriveAngle
        );

        driveSwerveStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void stop() {
        frontLeft.driveMotor.stopMotor();
        frontRight.driveMotor.stopMotor();
        rearLeft.driveMotor.stopMotor();
        rearRight.driveMotor.stopMotor();
    }

    public void driveSwerveStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MoPrefs.maxDriveSpeed.get());

        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

    public boolean isMoving() {
        return frontLeft.driveMotor.getSelectedSensorVelocity() /  MoPrefs.flDriveMtrScale.get() < MOVE_RATE_CUTOFF
            && frontRight.driveMotor.getSelectedSensorVelocity() /  MoPrefs.frDriveMtrScale.get() < MOVE_RATE_CUTOFF
            && rearLeft.driveMotor.getSelectedSensorVelocity() /  MoPrefs.rlDriveMtrScale.get() < MOVE_RATE_CUTOFF
            && rearRight.driveMotor.getSelectedSensorVelocity() /  MoPrefs.rrDriveMtrScale.get() < MOVE_RATE_CUTOFF;
    }

    @Override
    public void periodic() {
        frontLeft.periodic();
        frontRight.periodic();
        rearLeft.periodic();
        rearRight.periodic();

        if(DriverStation.isDisabled()) {
            this.stop();
        }
    }
}
