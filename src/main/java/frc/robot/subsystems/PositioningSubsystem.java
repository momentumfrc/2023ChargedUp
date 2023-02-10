package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Limelight;
import frc.robot.utils.MoShuffleboard;

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

    public Pose2d getRobotPose() {
		return robotPose;
	}

	public void setRobotPose(Pose3d pose) {
		Pose2d pose2d = pose.toPose2d();

		if(this.didEstablishInitialPosition.getBoolean(false)
			&& this.odometry.getPoseMeters().getTranslation().getDistance(pose2d.getTranslation()) > POSITION_MAX_ACCEPTABLE_UPDATE_DELTA)
		{
			return;
		}
		this.didEstablishInitialPosition.setBoolean(true);
		this.odometry.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose2d);
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
			this.setRobotPose(pose);
		});

        robotPose = odometry.update(gyro.getRotation2d(), drive.getWheelPositions());
		field.setRobotPose(robotPose);
    }
}
