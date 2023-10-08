package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;

public class PathFollowingUtils {
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 0.5);

    public static Command getFollowTrajectoryCommand(
        DriveSubsystem drive, PositioningSubsystem positioning,
        PathPlannerTrajectory trajectory, boolean shouldAssumeRobotIsAtStart
    ) {
        Command driveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            positioning::getRobotPose,
            drive.kinematics,
            drive.xPathController,
            drive.yPathController,
            drive.rotPathController,
            drive::driveSwerveStates,
            true,
            drive
        );

        if(shouldAssumeRobotIsAtStart) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(trajectory.getInitialState(), DriverStation.getAlliance());
                    positioning.setRobotPose(new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
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
