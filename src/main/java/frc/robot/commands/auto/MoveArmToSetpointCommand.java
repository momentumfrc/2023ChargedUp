package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public class MoveArmToSetpointCommand extends HoldArmSetpointCommand {
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

    public MoveArmToSetpointCommand(ArmSubsystem arms, ArmSetpoint setpoint) {
        super(arms, setpoint);
    }

    private boolean atTarget() {
        ArmPosition currPos = arms.getPosition();
        ArmPosition target = manager.getSetpoint(targetSetpoint);
        return Math.abs(currPos.shoulderRotations - target.shoulderRotations) < ON_TARGET_ZONE
            && Math.abs(currPos.wristRotations - target.wristRotations) < ON_TARGET_ZONE;
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = this.atTarget();
        if(atTarget) {
            return targetTimer.hasElapsed(ON_TARGET_TIME);
        } else {
            targetTimer.restart();
            return false;
        }
    }
}
