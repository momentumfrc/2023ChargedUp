package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public class TeleopArmCommand extends CommandBase {
    protected final ArmSubsystem arms;
    protected final Supplier<MoInput> inputSupplier;

    private ArmSetpoint lastSetpoint = ArmSetpoint.STOW;
    private boolean smartMotionPositionOverride = false;

    public TeleopArmCommand(ArmSubsystem arms, Supplier<MoInput> inputSupplier) {
        this.arms = arms;
        this.inputSupplier = inputSupplier;

        this.addRequirements(arms);
    }

    private void smartMotion(MoInput input) {
        Optional<ArmSetpoint> requestedSetpoint = input.getRequestedArmSetpoint();
        ArmMovementRequest requestedMovement = input.getArmMovementRequest();
        boolean shouldSaveSetpoint = input.getSaveArmSetpoint();

        if(requestedSetpoint.isPresent()) {
            if(shouldSaveSetpoint) {
                ArmSetpointManager.getInstance().setSetpoint(lastSetpoint, arms.getPosition());
            } else {
                smartMotionPositionOverride = false;
                lastSetpoint = requestedSetpoint.get();
            }
        }

        if(!requestedMovement.isZero()) {
            smartMotionPositionOverride = true;
        }

        ArmPosition requestedPosition = ArmSetpointManager.getInstance().getSetpoint(lastSetpoint);
        if(smartMotionPositionOverride) {
            arms.adjustVelocity(requestedMovement);
        } else {
            arms.adjustSmartPosition(requestedPosition);
        }
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();
        var controlMode = arms.armChooser.getSelected();
        switch(controlMode) {
            case FALLBACK_DIRECT_POWER:
                arms.adjustDirectPower(input.getArmMovementRequest());
            return;
            case DIRECT_VELOCITY:
                arms.adjustVelocity(input.getArmMovementRequest());
            return;
            case SMART_MOTION:
                this.smartMotion(input);
            return;
        }
    }

}
