package frc.robot.input;

import java.util.Optional;

import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public interface MoInput {
    public double getForwardSpeedRequest();
    public double getLeftSpeedRequest();
    public double getTurnRequest();

    public boolean getShouldUseSlowSpeed();
    public boolean getShouldIntake();
    public boolean getShouldExhaust();

    public ArmMovementRequest getArmMovementRequest();

    public Optional<ArmSetpoint> getRequestedArmSetpoint();
    public boolean getSaveArmSetpoint();
}
