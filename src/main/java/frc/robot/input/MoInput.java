package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface MoInput {
    public double getForwardSpeedRequest();
    public double getLeftSpeedRequest();
    public double getTurnRequest();

    public boolean getShouldUseSlowSpeed();
    public boolean getShouldMaintainWristParallel();

    public JoystickButton getRaiseArmButton();
    public JoystickButton getLowerArmButton();
    public JoystickButton getRetractWristButton();
    public JoystickButton getExtendWristButton();
}
