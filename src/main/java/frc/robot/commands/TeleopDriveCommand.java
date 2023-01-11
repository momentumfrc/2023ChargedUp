package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
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

        drive.driveCartesianFieldOriented(fwdRequest, leftRequest, turnRequest);
    }
}
