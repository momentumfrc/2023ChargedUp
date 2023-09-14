package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.utils.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.PathFollowingUtils;
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

    private static final double RESET_ENCODER_INTERVAL = 0.5;

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule rearLeft;
    public final SwerveModule rearRight;

    private final Timer resetEncoderTimer = new Timer();

    public final MoPIDF xPathController = new MoPIDF();
    public final MoPIDF yPathController = new MoPIDF();
    public final MoPIDF rotPathController = new MoPIDF();

    public final SwerveDriveKinematics kinematics;

    private final PIDTuner xPathTuner = TunerUtils.forMoPID(xPathController, "X Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);
    private final PIDTuner yPathTuner = TunerUtils.forMoPID(yPathController, "Y Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);
    private final PIDTuner rotPathTuner = TunerUtils.forMoPID(rotPathController, "Rot Path", !PathFollowingUtils.USE_HOLONOMIC_DRIVE);

    private final AHRS gyro;

    public DriveSubsystem(AHRS gyro) {
        this.gyro = gyro;

        this.frontLeft = new SwerveModule(
            "FL",
            new CANSparkMax(Constants.TURN_LEFT_FRONT.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_LEFT_FRONT.address),
            MoPrefs.flZero,
            MoPrefs.flScale,
            MoPrefs.flDriveMtrScale,
            true
        );

        this.frontRight = new SwerveModule(
            "FR",
            new CANSparkMax(Constants.TURN_RIGHT_FRONT.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_RIGHT_FRONT.address),
            MoPrefs.frZero,
            MoPrefs.frScale,
            MoPrefs.frDriveMtrScale,
            false
        );

        this.rearLeft = new SwerveModule(
            "RL",
            new CANSparkMax(Constants.TURN_LEFT_REAR.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_LEFT_REAR.address),
            MoPrefs.rlZero,
            MoPrefs.rlScale,
            MoPrefs.rlDriveMtrScale,
            false
        );

        this.rearRight = new SwerveModule(
            "RR",
            new CANSparkMax(Constants.TURN_RIGHT_REAR.address, MotorType.kBrushless),
            new WPI_TalonFX(Constants.DRIVE_RIGHT_REAR.address),
            MoPrefs.rrZero,
            MoPrefs.rrScale,
            MoPrefs.rrDriveMtrScale,
            false
        );

        resetEncoderTimer.start();

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
        var positions = new SwerveModulePosition[] {
            // TODO
            new SwerveModulePosition(0, Rotation2d.fromRadians(0)),
            new SwerveModulePosition(0, Rotation2d.fromRadians(0)),
            new SwerveModulePosition(0, Rotation2d.fromRadians(0)),
            new SwerveModulePosition(0, Rotation2d.fromRadians(0))
        };

        return positions;
    }

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest) {
        this.driveCartesian(fwdRequest, leftRequest, turnRequest, new Rotation2d());
    }

    public void driveCartesian(double fwdRequest, double leftRequest, double turnRequest, Rotation2d fieldOrientedDriveAngle) {
        double maxLinearSpeed = MoPrefs.maxDriveSpeed.get();
        double maxAngularSpeed = MoPrefs.maxTurnSpeed.get();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fwdRequest * maxLinearSpeed,
            leftRequest * maxLinearSpeed,
            turnRequest * maxAngularSpeed,
            fieldOrientedDriveAngle.unaryMinus()
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
        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

    public void driveDifferentialWheelSpeeds(double leftMps, double rightMps) {
        // TODO
    }

    // TODO
    public boolean isMoving() {
        return false;
    }

    public void resetRelativeEncoders() {
        frontLeft.setRelativePosition();
        frontRight.setRelativePosition();
        rearLeft.setRelativePosition();
        rearRight.setRelativePosition();
    }

    @Override
    public void periodic() {
        if(resetEncoderTimer.advanceIfElapsed(RESET_ENCODER_INTERVAL)) {
            resetRelativeEncoders();
        }

        if(DriverStation.isDisabled()) {
            this.stop();
        }
    }
}
