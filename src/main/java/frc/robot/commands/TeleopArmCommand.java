package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;
import frc.robot.utils.MoPrefs.Pref;

public class TeleopArmCommand extends CommandBase {
    private final ArmSubsystem arms;
    private final Supplier<MoInput> inputSupplier;

    private Pref<Double> rampTime = MoPrefs.armRampTime;

    private ArmSetpoint lastSetpoint = ArmSetpoint.STOW;
    private boolean smartMotionPositionOverride = false;

    private SlewRateLimiter shoulderLimiter;
    private SlewRateLimiter wristLimiter;

    public TeleopArmCommand(ArmSubsystem arms, Supplier<MoInput> inputSupplier) {
        this.arms = arms;
        this.inputSupplier = inputSupplier;

        rampTime.subscribe(rampTime -> {
            double slewRate = 1.0 / rampTime;

            shoulderLimiter = new SlewRateLimiter(slewRate);
            wristLimiter = new SlewRateLimiter(slewRate);
        }, true);

        addRequirements(arms);
    }

    private ArmMovementRequest getLimitedMovementRequest(MoInput input) {
        ArmMovementRequest requestedMovement = input.getArmMovementRequest();
        return new ArmMovementRequest(
            shoulderLimiter.calculate(requestedMovement.shoulderPower),
            wristLimiter.calculate(requestedMovement.wristPower)
        );
    }

    private void smartMotion(MoInput input) {
        Optional<ArmSetpoint> requestedSetpoint = input.getRequestedArmSetpoint();
        ArmMovementRequest requestedMovement = getLimitedMovementRequest(input);
        boolean shouldSaveSetpoint = input.getSaveArmSetpoint();

        if(requestedSetpoint.isPresent()) {
            if(shouldSaveSetpoint) {
                ArmSetpointManager.getInstance().setSetpoint(requestedSetpoint.get(), arms.getPosition());
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

        if(input.getReZeroArms()) {
            arms.reZero();
        }

        switch(controlMode) {
            case FALLBACK_DIRECT_POWER:
                arms.adjustDirectPower(getLimitedMovementRequest(input));
            return;
            case DIRECT_VELOCITY:
                arms.adjustVelocity(getLimitedMovementRequest(input));
            return;
            case SMART_MOTION:
                this.smartMotion(input);
            return;
        }
    }

}
