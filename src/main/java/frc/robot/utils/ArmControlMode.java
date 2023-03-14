package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.input.MoInput;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public abstract class ArmControlMode implements Nameable {
    public static final List<ArmControlMode> ALL_CONTROL_TYPES = new ArrayList<>();
    public final String name;

    protected ArmControlMode(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return this.name;
    }

    public abstract void apply(MoInput input, ArmSubsystem arms);

    public boolean allowsAutonomousSmartMotion() {
        return false;
    }

    static {
        ALL_CONTROL_TYPES.add(new DirectPower());
        ALL_CONTROL_TYPES.add(new DirectVelocity());
        ALL_CONTROL_TYPES.add(new FineControlSmartMotion());
        ALL_CONTROL_TYPES.add(new CraneSmartMotion());
    }

    public static class DirectPower extends ArmControlMode {
        public DirectPower() {
            super("Direct Power (Fallback)");
        }

        @Override
        public void apply(MoInput input, ArmSubsystem arms) {
            arms.adjustDirectPower(arms.limitedMovementOf(input.getDirectArmMovementRequest()));
        }
    }

    public static class DirectVelocity extends ArmControlMode {
        public DirectVelocity() {
            super("Direct Velocity");
        }

        @Override
        public void apply(MoInput input, ArmSubsystem arms) {
            arms.adjustVelocity(arms.limitedMovementOf(input.getDirectArmMovementRequest()));
        }
    }

    public static class FineControlSmartMotion extends ArmControlMode {
        private ArmSetpoint lastSetpoint = ArmSetpoint.STOW;
        private boolean smartMotionPositionOverride = false;

        public FineControlSmartMotion() {
            super("Fine-Control Smart Motion");
        }

        @Override
        public void apply(MoInput input, ArmSubsystem arms) {
            Optional<ArmSetpoint> requestedSetpoint = input.getFineControlArmSetpoint();
            ArmMovementRequest requestedMovement = arms.limitedMovementOf(input.getDirectArmMovementRequest());
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
        public boolean allowsAutonomousSmartMotion() {
            return true;
        }
    }

    public static class CraneSmartMotion extends ArmControlMode {
        private ArmSetpoint lastSetpoint = ArmSetpoint.STOW;

        public CraneSmartMotion() {
            super("Crane Smart Motion");
        }

        @Override
        public void apply(MoInput input, ArmSubsystem arms) {
            Optional<ArmSetpoint> setpoint = input.getCraneControlArmSetpoint();
            ArmPosition posMultiplier = input.getCraneControlArmMovementRequest();

            boolean shouldSaveSetpoint = input.getSaveArmSetpoint();

            if (setpoint.isPresent()) {
                if (shouldSaveSetpoint) {
                    ArmSetpointManager.getInstance().setSetpoint(lastSetpoint, arms.getPosition());
                } else {
                    lastSetpoint = setpoint.get();
                }
            }

            ArmPosition requestedPosition = ArmSetpointManager.getInstance().getSetpoint(lastSetpoint);
            requestedPosition = new ArmPosition(
                requestedPosition.shoulderRotations * posMultiplier.shoulderRotations,
                requestedPosition.wristRotations * posMultiplier.wristRotations);

            arms.adjustSmartPosition(requestedPosition);
        }

        @Override
        public boolean allowsAutonomousSmartMotion() {
            return true;
        }
    }
}
