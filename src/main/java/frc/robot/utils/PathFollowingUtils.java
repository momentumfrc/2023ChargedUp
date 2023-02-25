package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;

public class PathFollowingUtils {
    public static final boolean USE_HOLONOMIC_DRIVE = false;

    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4, 0.5);

    public static Command getFollowTrajectoryCommand(
        DriveSubsystem drive, PositioningSubsystem positioning,
        PathPlannerTrajectory trajectory, boolean shouldAssumeRobotIsAtStart
    ) {
        Command driveControllerCommand;
        if(USE_HOLONOMIC_DRIVE) {
            driveControllerCommand = new PPMecanumControllerCommand(
                trajectory,
                positioning::getReverseRobotPose,
                positioning.kinematics,
                drive.xPathController,
                drive.yPathController,
                drive.rotPathController,
                MoPrefs.maxDriveRpm.get() / DriveSubsystem.REVOLUTIONS_PER_METER,
                drive::driveWheelSpeeds,
                true,
                drive
            );
        } else {
            driveControllerCommand = new PPRamseteCommand(
                trajectory,
                positioning::getReverseRobotPose,
                new RamseteController(),
                positioning.getDifferentialKinematics(),
                drive::driveDifferentialWheelSpeeds,
                true,
                drive
            );
        }

        if(shouldAssumeRobotIsAtStart) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drive.resetMaintainHeading();
                    PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
                    if(USE_HOLONOMIC_DRIVE) {
                        positioning.setRobotPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation.rotateBy(Rotation2d.fromRotations(0.5))));
                    } else {
                        positioning.setRobotPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.poseMeters.getRotation().rotateBy(Rotation2d.fromRotations(0.5))));
                    }
                }),
                driveControllerCommand
            );
        } else {
            return driveControllerCommand;
        }
    }

    public static Command getFollowTrajectoryCommand(
        DriveSubsystem drive, PositioningSubsystem positioning,
        String trajectoryName, boolean shouldAssumeRobotIsAtStart
    ) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, PATH_CONSTRAINTS);
        return getFollowTrajectoryCommand(drive, positioning, trajectory, shouldAssumeRobotIsAtStart);
    }
}
