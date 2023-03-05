package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MoPrefs;

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
        double intakeSpeed = MoPrefs.intakeSpeed.get();
        double output = 0;
        if(input.getShouldIntake())
            output += intakeSpeed;
        if(input.getShouldExhaust())
            output -= intakeSpeed;
        this.intake.runIntake(output);
    }
}
