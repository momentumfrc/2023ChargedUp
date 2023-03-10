package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public class MoveArmToPositionCommand extends HoldArmPositionCommand {
    /**
     * The duration (in seconds) for which the arm subsystem must be within ON_TARGET_ZONE of the
     * requested position before the arms are considered to be at the requested position.
     */
    private static final double ON_TARGET_TIME = 0.5;

    /**
     * The distance (in rotations) which the arm subsystem must be within of the
     * requested position before the arms are considered to be at the requested position.
     */
    private static final double ON_TARGET_ZONE = 0.05;

    private final Timer targetTimer = new Timer();

    public MoveArmToPositionCommand(ArmSubsystem arms, ArmPosition position) {
        super(arms, position);
    }

    public static MoveArmToPositionCommand fromSetpoint(ArmSubsystem arms, ArmSetpoint setpoint) {
        return new MoveArmToPositionCommand(arms, ArmSetpointManager.getInstance().getSetpoint(setpoint));
    }

    private boolean atTarget() {
        ArmPosition currPos = arms.getPosition();
        return Math.abs(currPos.shoulderRotations - targetPosition.shoulderRotations) < ON_TARGET_ZONE
            && Math.abs(currPos.wristRotations - targetPosition.wristRotations) < ON_TARGET_ZONE;
    }

    @Override
    public boolean isFinished() {
        if(atTarget()) {
            return targetTimer.hasElapsed(ON_TARGET_TIME);
        } else {
            targetTimer.restart();
            return false;
        }
    }
}
