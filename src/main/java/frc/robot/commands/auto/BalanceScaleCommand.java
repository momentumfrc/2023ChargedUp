package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A simple bang-bang controller to automatically level the scale.
 */
public class BalanceScaleCommand extends CommandBase {
    private static final double MOVE_SPEED = 0.1;
    private static final double LEVEL_DEFINITION = 2.5; // degrees

    private final DriveSubsystem drive;
    private final AHRS navx;

    public BalanceScaleCommand(DriveSubsystem drive, AHRS navx) {
        this.drive = drive;
        this.navx = navx;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetMaintainHeading();
    }

    @Override
    public void execute() {
        // TODO: Verify if we need pitch or roll here
        double pitch = navx.getPitch();

        double moveRequest = 0;
        if(pitch < -LEVEL_DEFINITION) {
            moveRequest = -MOVE_SPEED;
        } else if(pitch > LEVEL_DEFINITION) {
            moveRequest = MOVE_SPEED;
        }

        drive.driveCartesian(moveRequest, 0, 0);
    }

}
