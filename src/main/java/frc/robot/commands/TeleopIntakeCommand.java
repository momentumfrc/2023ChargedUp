package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.IntakeSubsystem;

public class TeleopIntakeCommand extends CommandBase {
    private final MoInput input;
    private final IntakeSubsystem intake;

    public TeleopIntakeCommand(MoInput input, IntakeSubsystem intake) {
        this.input = input;
        this.intake = intake;

        this.addRequirements(intake);
    }

    @Override
    public void execute() {
        this.intake.runIntake(
            (input.getShouldIntake() ? 1 : 0) +
            (input.getShouldExhaust() ? -1 : 0)
        );
    }
}
