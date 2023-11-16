package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.utils.PathFollowingUtils;

public class AutoAlignCommand extends CommandBase {

    private Pose2d target = new Pose2d(new Translation2d(2.04, 3.436), Rotation2d.fromDegrees(-16.04));
    private PositioningSubsystem posSub;
    private DriveSubsystem driSub;

    public AutoAlignCommand(PositioningSubsystem posSub, DriveSubsystem driSub) {

        this.posSub = posSub;
        this.driSub = driSub;

    }

    @Override
    public void initialize() {
        Pose2d current = posSub.getRobotPose();
        // Simple path without holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
        Rotation2d angle = target.getTranslation().minus(current.getTranslation()).getAngle();
        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
            new PathConstraints(3, 3),
            new PathPoint(current.getTranslation(), angle, current.getRotation()), // position, heading, rotation
            new PathPoint(target.getTranslation(), angle, target.getRotation()) // position, heading, rotation
        );

        Command followCommand = PathFollowingUtils.getFollowTrajectoryCommand(driSub, posSub, traj1, false);

        followCommand.schedule();
    }

    @Override
    public boolean isFinished(){

        return true;
    }


}
