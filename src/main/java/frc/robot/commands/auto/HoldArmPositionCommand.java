package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmControlMode;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public class HoldArmPositionCommand extends CommandBase {

    protected final ArmPosition targetPosition;
    protected final ArmSubsystem arms;

    public HoldArmPositionCommand(ArmSubsystem arms, ArmPosition position) {
        this.targetPosition = position;
        this.arms = arms;
    }

    public static HoldArmPositionCommand fromSetpoint(ArmSubsystem arms, ArmSetpoint setpoint) {
        return new HoldArmPositionCommand(arms, ArmSetpointManager.getInstance().getSetpoint(setpoint));
    }

    @Override
    public void execute() {
        ArmControlMode mode = arms.armChooser.getSelected();
        if(!mode.allowsAutonomousSmartMotion()) {
            DriverStation.reportWarning("Attempt to autonomously move arms when SmartMotion is disabled", false);
            arms.stop();
            return;
        }

        arms.adjustSmartPosition(targetPosition);
    }
}
