package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.Pref;

public class TeleopDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final PositioningSubsystem positioning;
    private final MoInput input;

    private Pref<Double> rampTime = MoPrefs.driveRampTime;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter leftLimiter;
    private SlewRateLimiter turnLimiter;

    public TeleopDriveCommand(DriveSubsystem drive, PositioningSubsystem positioning, MoInput input) {
        this.drive = drive;
        this.positioning = positioning;
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
    public void initialize() {
        // We need to make sure the robot doesn't try to snap back to zero if it was pushed
        // around while it was disabled.
        // Since runsWhenDisabled() returns false, this command will be canceled
        // whenever the robot is disabled. Then, when the robot is re-enabled, it will be
        // rescheduled (since it is the default command of DriveSubsystem) and init will
        // be called again.
        // Thus, every time teleop is started, this method will be called and the drive
        // will reset its heading to the current value.
        drive.resetMaintainHeading();
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

        Rotation2d fieldOrientedDriveAngle;

        if(this.drive.shouldDriveFieldOriented.getBoolean(true)) {
            fieldOrientedDriveAngle = positioning.getRobotPose().getRotation();
        } else {
            fieldOrientedDriveAngle = new Rotation2d();
        }
        drive.driveCartesian(fwdRequest, leftRequest, turnRequest, fieldOrientedDriveAngle);

    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
