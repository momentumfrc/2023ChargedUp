package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopArmCommand extends CommandBase {
    private final ArmSubsystem arms;
    private final Supplier<MoInput> inputSupplier;

    public TeleopArmCommand(ArmSubsystem arms, Supplier<MoInput> inputSupplier) {
        this.arms = arms;
        this.inputSupplier = inputSupplier;

        addRequirements(arms);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        arms.armChooser.getSelected().apply(input, this.arms);;
    }
}
