package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;

public class PathFollowingUtils {
    public static Command getFollowTrajectoryCommand(
        DriveSubsystem drive, PositioningSubsystem positioning,
        PathPlannerTrajectory trajectory, boolean shouldAssumeRobotIsAtStart
    ) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(shouldAssumeRobotIsAtStart) {
                    PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
                    positioning.setRobotPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
                }
            }),
            new PPMecanumControllerCommand(
                trajectory,
                positioning::getRobotPose,
                positioning.kinematics,
                drive.xPathController,
                drive.yPathController,
                drive.rotPathController,
                MoPrefs.maxDriveRpm.get(),
                drive::driveWheelSpeeds,
                true,
                drive, positioning
            )
        );
    }

    public static Command getFollowTrajectoryCommand(
        DriveSubsystem drive, PositioningSubsystem positioning,
        String trajectoryName, boolean shouldAssumeRobotIsAtStart
    ) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, new PathConstraints(2, 1.5));
        return getFollowTrajectoryCommand(drive, positioning, trajectory, shouldAssumeRobotIsAtStart);
    }
}
