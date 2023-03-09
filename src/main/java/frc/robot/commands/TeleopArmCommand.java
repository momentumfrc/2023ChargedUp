package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.MoPrefs;

public class TeleopArmCommand extends CommandBase {
    protected final ArmSubsystem arms;
    protected final MoInput input;

    public TeleopArmCommand(ArmSubsystem arms, MoInput input) {
        this.arms = arms;
        this.input = input;

        this.addRequirements(arms);
    }

    @Override
    public void execute() {
        var controlMode = arms.armChooser.getSelected();
        switch(controlMode) {
            case FALLBACK_DIRECT_POWER:
                arms.adjustDirectPower(input.getDirectShoulderRequest(), input.getDirectWristRequest() * MoPrefs.wristFallbackPower.get());
            return;
            case DIRECT_VELOCITY:
                arms.adjustVelocity(
                    input.getDirectShoulderRequest() * MoPrefs.shoulderMaxRpm.get(),
                    input.getDirectWristRequest() * MoPrefs.wristMaxRpm.get()
                );
            return;
            case SMART_MOTION:
                var requestedPosition = input.getArmPositionRequest();
                arms.adjustSmartPosition(
                    requestedPosition.shoulderRotations,
                    requestedPosition.wristRotations
                );
            return;
        }
    }

}
