package frc.robot.input;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public interface MoInput {
    public Pair<Double, Double> getMoveRequest();
    public double getTurnRequest();

    public boolean getShouldUseSlowSpeed();
    public boolean getShouldDriveAligned();
    public boolean getShouldIntake();
    public boolean getShouldExhaust();

    public boolean getShouldAlignCubes();
    public boolean getShouldAlignCones();

    public boolean getShouldBrake();

    public ArmMovementRequest getArmMovementRequest();

    public Optional<ArmSetpoint> getRequestedArmSetpoint();
    public boolean getSaveArmSetpoint();

    public boolean getReZeroArms();
    public boolean getReZeroGyro();
    public boolean getShouldBalance();
}
