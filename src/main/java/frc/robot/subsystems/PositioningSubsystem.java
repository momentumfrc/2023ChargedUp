package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Limelight;
import frc.robot.utils.MoShuffleboard;

/**
 * Subsystem that determines the robot's position on the field.
 * <p>
 * Note: the robot's pose is defined in alliance-relative coordinates.
 * This means that the right-hand side of the alliance wall is considered the origin. This
 * simplifies autonomous by allowing the same coordinates to be used no matter the alliance in
 * which the team is currently participating. (Well, since the field is mirrored this year,
 * this logic doesn't quite work. But the pathing library already takes care of the mirroring
 * for us, so as long as our poses are alliance-relative, it all works out.)
 * <p>
 * If you absolutely must have the absolute position of the robot in field coordinates
 * (instead of alliance coordinates), use the {@link #getAbsoluteRobotPose} method. But this
 * should really only be needed for displaying the robot's position on the shuffleboard, and not for
 * any autonomous logic.
 */
public class PositioningSubsystem extends SubsystemBase {
    /**
     * The distance, in meters, of a wheel from the center of the robot towards the
     * front of the robot.
     */
    private static final double WHEEL_FWD_POS = 0.25797; // 0.2664394;

    /**
     * The distance, in meters, of a wheel from the center of the robot towards the
     * left side of the robot.
     */
    private static final double WHEEL_LEFT_POS = 0.288131; // 0.294386;

    /**
     * The maximum acceptable distance, in meters, between a limelight position update and the
     * robot's current odometry.
     */
    private static final double POSITION_MAX_ACCEPTABLE_UPDATE_DELTA = 5;

    /**
     * The size of the field, in meters. Used to transform alliance-relative coordinates
     * into field-relative coordinates.
     */
    private static final Translation2d fieldSize = new Translation2d(16.54175, 8.0137);
    /**
     * The limelight. Should be used by auto scoring commands for fine targeting.
     */
    public final Limelight limelight = new Limelight();

    private Pose2d robotPose = new Pose2d();

    private Field2d field = MoShuffleboard.getInstance().field;

    private GenericEntry didEstablishInitialPosition = MoShuffleboard.getInstance().matchTab.add("Initial Position", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    private GenericEntry shouldUseAprilTags = MoShuffleboard.getInstance().settingsTab.add("Detect AprilTags", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    private final AHRS gyro;
    private final DriveSubsystem drive;

    public final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(WHEEL_FWD_POS, WHEEL_LEFT_POS),
        new Translation2d(WHEEL_FWD_POS, -WHEEL_LEFT_POS),
        new Translation2d(-WHEEL_FWD_POS, WHEEL_LEFT_POS),
        new Translation2d(-WHEEL_FWD_POS, -WHEEL_LEFT_POS)
    );
    private MecanumDriveOdometry odometry;

    public PositioningSubsystem(AHRS ahrs, DriveSubsystem drive) {
        this.gyro = ahrs;
        this.drive = drive;

        odometry = new MecanumDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            drive.getWheelPositions()
        );
    }

    public DifferentialDriveKinematics getDifferentialKinematics() {
        return new DifferentialDriveKinematics(WHEEL_LEFT_POS * 2);
    }

    /**
     * Get robot pose in alliance coordinates.
     * <p>
     * Note that the robot always assumes its origin is in the right corner of its alliance.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    public Pose2d getReverseRobotPose() {
        return new Pose2d(robotPose.getTranslation(), robotPose.getRotation().plus(Rotation2d.fromRotations(0.5)));
    }

    /**
     * Get the robot pose in field coordinates.
     */
    public Pose2d getAbsoluteRobotPose() {
        if(DriverStation.getAlliance() == Alliance.Blue) {
            return robotPose;
        }
        Pose2d alliancePose = robotPose;
        Translation2d translation = fieldSize.minus(alliancePose.getTranslation());
        Rotation2d rotation = robotPose.getRotation().rotateBy(Rotation2d.fromRotations(0.5));
        return new Pose2d(translation, rotation);
    }

    public void setRobotPose(Pose2d pose) {
        if(this.didEstablishInitialPosition.getBoolean(false)
            && this.odometry.getPoseMeters().getTranslation().getDistance(pose.getTranslation()) > POSITION_MAX_ACCEPTABLE_UPDATE_DELTA)
        {
            return;
        }
        this.didEstablishInitialPosition.setBoolean(true);
        this.odometry.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose);
    }


    @Override
    public void periodic() {
        limelight.periodic();

        limelight.getRobotPose().ifPresent(pose -> {
            if(!shouldUseAprilTags.getBoolean(true)) {
                return;
            }
            if(drive.isMoving()) {
                return;
            }
            this.setRobotPose(pose.toPose2d());
        });

        robotPose = odometry.update(gyro.getRotation2d(), drive.getWheelPositions());
        field.setRobotPose(getAbsoluteRobotPose());
    }
}
