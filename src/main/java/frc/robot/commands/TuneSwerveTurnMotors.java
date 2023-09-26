package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.DriveSubsystem;

public class TuneSwerveTurnMotors extends CommandBase {

    private final DriveSubsystem drive;
    private final Supplier<MoInput> inputSupplier;

    public TuneSwerveTurnMotors(DriveSubsystem drive, Supplier<MoInput> inputSupplier) {
        this.drive = drive;
        this.inputSupplier = inputSupplier;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        DriverStation.reportWarning("ENTERING DRIVE TURN CALIBRATION", false);

        drive.doResetEncoders = false;
    }

    @Override
    public void execute() {
        double fwdRequest = inputSupplier.get().getForwardSpeedRequest();
        double turnRequest = inputSupplier.get().getLeftSpeedRequest();

        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(-turnRequest, -fwdRequest));

        drive.frontLeft.drive(state);
        drive.frontRight.drive(state);
        drive.rearLeft.drive(state);
        drive.rearRight.drive(state);
    }

    @Override
    public void end(boolean wasInterrupted) {
        DriverStation.reportWarning("FINISHING DRIVE TURN CALIBRATION", false);

        drive.doResetEncoders = true;
    }

}
