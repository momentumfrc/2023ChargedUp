package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;

public abstract class TeleopArmCommand extends CommandBase {
    private final ArmSubsystem arms;
    private final MoInput input;

    protected TeleopArmCommand(ArmSubsystem arms, MoInput input) {
        this.arms = arms;
        this.input = input;

        this.addRequirements(arms);
    }

    @Override
    public void execute() {
        this.arms.adjustShoulders(this.getShoulderPower(this.input));
        this.arms.adjustWrist(this.getWristPower(this.input));
    }

    public abstract double getShoulderPower(MoInput input);

    public abstract double getWristPower(MoInput input);

    public static class Direct extends TeleopArmCommand {
        public Direct(ArmSubsystem arms, MoInput input) {
            super(arms, input);
        }

        @Override
        public double getShoulderPower(MoInput input) {
            return input.getDirectShoulderRequest();
        }

        @Override
        public double getWristPower(MoInput input) {
            return input.getDirectWristRequest();
        }
    }

    // TODO: public static class Combo extends TeleopArmCommand {}
}
