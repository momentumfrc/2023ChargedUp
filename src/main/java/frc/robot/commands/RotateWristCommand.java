package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class RotateWristCommand extends CommandBase {
    private final ArmSubsystem arms;
    private final double power;

    public RotateWristCommand(ArmSubsystem arms, double power) {
        this.arms = arms;
        this.power = power;
    }

    @Override
    public void execute() {
        this.arms.adjustWrist(power);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.arms.idleWrist();
    }
}
