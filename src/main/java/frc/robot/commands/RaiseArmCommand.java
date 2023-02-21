package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.MoPrefs;

public class RaiseArmCommand extends CommandBase {
    private final ArmSubsystem arms;
    private final double power;
    private final BooleanSupplier shouldBeParallel;
    private boolean wasParallel;

    public RaiseArmCommand(ArmSubsystem arms, double power, BooleanSupplier parallel) {
        this.arms = arms;
        this.power = power;
        this.shouldBeParallel = parallel;
    }

    @Override
    public void execute() {
        this.arms.adjustArm(power);

        boolean isParallel = this.shouldBeParallel.getAsBoolean();
        if (this.wasParallel && !isParallel) {
            this.arms.idleWrist();
        } else if (isParallel) {
            this.arms.adjustWrist(power * MoPrefs.armJointSetpointRatio.get());
        }

        this.wasParallel = isParallel;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.arms.idleArm();

        if (this.wasParallel) {
            this.arms.idleWrist();
        }
    }
}
