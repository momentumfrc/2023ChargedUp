package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MoPrefs;

public class TeleopIntakeCommand extends CommandBase {
    private final Supplier<MoInput> inputSupplier;
    private final IntakeSubsystem intake;

    public TeleopIntakeCommand(IntakeSubsystem intake, Supplier<MoInput> inputSupplier) {
        this.inputSupplier = inputSupplier;
        this.intake = intake;

        this.addRequirements(intake);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        double intakeSpeed = MoPrefs.intakeSpeed.get();
        double output = 0;
        if(input.getShouldIntake())
            output += intakeSpeed;
        if(input.getShouldExhaust())
            output -= intakeSpeed;
        this.intake.runIntake(output);
    }
}
