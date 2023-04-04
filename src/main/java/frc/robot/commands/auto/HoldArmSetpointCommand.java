package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmControlMode;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.utils.ArmSetpointManager;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

public class HoldArmSetpointCommand extends CommandBase {

    protected final ArmSetpoint targetSetpoint;
    protected final ArmSubsystem arms;
    protected final ArmSetpointManager manager = ArmSetpointManager.getInstance();

    public HoldArmSetpointCommand(ArmSubsystem arms, ArmSetpoint setpoint) {
        this.targetSetpoint = setpoint;
        this.arms = arms;

        addRequirements(arms);
    }

    @Override
    public void execute() {
        ArmControlMode mode = arms.armChooser.getSelected();
        if(mode != ArmControlMode.SMART_MOTION) {
            DriverStation.reportWarning("Attempt to autonomously move arms when SmartMotion is disabled", false);
            arms.stop();
            return;
        }

        arms.adjustSmartPosition(manager.getSetpoint(targetSetpoint));
    }
}
