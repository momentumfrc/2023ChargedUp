package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.Pref;

public class TeleopDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final MoInput input;

    private Pref<Double> rampTime = MoPrefs.driveRampTime;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter leftLimiter;
    private SlewRateLimiter turnLimiter;

    public TeleopDriveCommand(DriveSubsystem drive, MoInput input) {
        this.drive = drive;
        this.input = input;

        rampTime.subscribe(rampTime -> {
            double slewRate = 1.0 / rampTime;

            fwdLimiter = new SlewRateLimiter(slewRate);
            leftLimiter = new SlewRateLimiter(slewRate);
            turnLimiter = new SlewRateLimiter(slewRate);
        }, true);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double fwdRequest = input.getForwardSpeedRequest();
        double leftRequest = input.getLeftSpeedRequest();
        double turnRequest = input.getTurnRequest();

        if(input.getShouldUseSlowSpeed()) {
            double slowSpeed = MoPrefs.driveSlowSpeed.get();
            fwdRequest *= slowSpeed;
            leftRequest *= slowSpeed;
            turnRequest *= slowSpeed;
        }

        fwdRequest = fwdLimiter.calculate(fwdRequest);
        leftRequest = leftLimiter.calculate(leftRequest);
        turnRequest = turnLimiter.calculate(turnRequest);

        drive.driveCartesian(fwdRequest, leftRequest, turnRequest);
    }
}
