package frc.robot.subsystems;

import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MoShuffleboard;

public class PositioningSubsystem extends SubsystemBase {
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

	/**
	 * The maximum acceptable distance, in meters, between a limelight position update and the
	 * robot's current odometry.
	 */
	private static final double POSITION_MAX_ACCEPTABLE_UPDATE_DELTA = 5;

	private final LimelightTableAdapter limelight = new LimelightTableAdapter();
	private Pose2d robotPose = new Pose2d();
	private double poseUncertainty = 0;

	private long lastTagCycleTimestamp = System.currentTimeMillis();
	private long lastOdoCycleTimestamp = System.currentTimeMillis();

	private Field2d field = MoShuffleboard.getInstance().field;

	private boolean shouldLights = false;

	private GenericEntry didEstablishInitialPosition = MoShuffleboard.getInstance().matchTab.add("Initial Position", false).getEntry();

	private GenericEntry shouldUseAprilTags = MoShuffleboard.getInstance().settingsTab.add("Detect AprilTags", true).getEntry();

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


    @Override
    public void periodic() {
		limelight.periodic();

		limelight.setLight(shouldLights);

		limelight.ifPose(pose -> {
			if(!shouldUseAprilTags.getBoolean(true)) {
				return;
			}
			if(drive.isMoving()) {
				return;
			}
			long time = System.currentTimeMillis();
			this.setRobotPose((int)(time - this.lastTagCycleTimestamp), -1, pose);
		});

        robotPose = odometry.update(gyro.getRotation2d(), drive.getWheelPositions());
		field.setRobotPose(robotPose);
    }

	public Pose2d getRobotPose() {
		return robotPose;
	}

	public void setRobotPose(int deltaMs, double certainty, Pose3d pose) {
		this.poseUncertainty += (certainty / deltaMs);

		Pose2d pose2d = pose.toPose2d();

		if(this.didEstablishInitialPosition.getBoolean(false)
			&& this.odometry.getPoseMeters().getTranslation().getDistance(pose2d.getTranslation()) > POSITION_MAX_ACCEPTABLE_UPDATE_DELTA)
		{
			return;
		}
		this.didEstablishInitialPosition.setBoolean(true);
		this.odometry.resetPosition(gyro.getRotation2d(), drive.getWheelPositions(), pose2d);
	}

	public enum LimelightPipeline {
		FIDUCIAL, REFLECTORS
	}

    // Limelight values are automatically put into a network
	// table, this is a wrapper around that
	public static class LimelightTableAdapter {
		private static final double[] emptyPose = new double[6];

		private boolean hasDetection = false;
		private Pose3d fieldPose = new Pose3d();
		private LimelightPipeline pipeline = LimelightPipeline.FIDUCIAL;

		private NetworkTable getTable() {
			return NetworkTableInstance.getDefault().getTable("limelight");
		}

		private boolean isPoseValid(double[] pose) {
			return pose.length >= 6 && (pose[0] != 0 || pose[1] != 0 || pose[2] != 0 || pose[3] != 0 || pose[4] != 0 || pose[5] != 0);
		}

		public void periodic() {
			NetworkTable table = this.getTable();

			this.hasDetection = table.getEntry("tv").getNumber(0).doubleValue() > 0;
			double[] pose = table.getEntry("botpose_wpiblue").getDoubleArray(emptyPose);
			if (isPoseValid(pose)) {
				double rad = Math.PI / 180;
				this.fieldPose = new Pose3d(
					new Translation3d(pose[0], pose[1], pose[2]),
					new Rotation3d(pose[3] * rad, pose[4] * rad, pose[5] * rad));
			}
		}

		public boolean hasDetection() {
			return this.hasDetection;
		}

		public void ifPose(Consumer<Pose3d> action) {
			if (this.pipeline == LimelightPipeline.FIDUCIAL && this.hasDetection()) {
				action.accept(this.fieldPose);
			}
		}

		public void setLight(boolean on) {
			this.getTable().getEntry("ledMode").setNumber(on ? 3 : 1);
		}

		public void setPipeline(LimelightPipeline pipeline) {
			this.pipeline = pipeline;
			this.getTable().getEntry("pipeline").setNumber(pipeline.ordinal());
		}
	}
}
