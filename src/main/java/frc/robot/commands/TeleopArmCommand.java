package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;

public abstract class TeleopArmCommand extends CommandBase {
    protected final ArmSubsystem arms;
    protected final MoInput input;

    protected TeleopArmCommand(ArmSubsystem arms, MoInput input) {
        this.arms = arms;
        this.input = input;

        this.addRequirements(arms);
    }

    public static class Direct extends TeleopArmCommand {
        private final boolean pid;

        public Direct(ArmSubsystem arms, MoInput input, boolean pid) {
            super(arms, input);

            this.pid = pid;
        }

        @Override
        public void execute() {
            this.arms.adjustShoulders(pid, input.getDirectShoulderRequest());
            this.arms.adjustWrist(pid, input.getDirectWristRequest());
        }
    }

    // TODO: public static class Combo extends TeleopArmCommand {}
}
