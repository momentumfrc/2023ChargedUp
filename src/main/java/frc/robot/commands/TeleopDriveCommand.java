package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
    private double SLOW_SPEED = 0.5;

    private final DriveSubsystem drive;
    private final MoInput input;

    public TeleopDriveCommand(DriveSubsystem drive, MoInput input) {
        this.drive = drive;
        this.input = input;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double fwdRequest = input.getForwardSpeedRequest();
        double leftRequest = input.getLeftSpeedRequest();
        double turnRequest = input.getTurnRequest();

        if(input.getShouldUseSlowSpeed()) {
            fwdRequest *= SLOW_SPEED;
            leftRequest *= SLOW_SPEED;
            turnRequest *= SLOW_SPEED;
        }

        drive.driveCartesian(fwdRequest, leftRequest, turnRequest);
    }
}
