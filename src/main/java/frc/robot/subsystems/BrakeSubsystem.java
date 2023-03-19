package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BrakeSubsystem extends SubsystemBase {
    private final DoubleSolenoid intakePiston = new DoubleSolenoid(
		PneumaticsModuleType.CTREPCM, Constants.EXTEND_BRAKE, Constants.RETRACT_BRAKE);

    private final PneumaticsControlModule pcm = new PneumaticsControlModule();
    private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    public void setBrakesExtended(boolean extended) {
        intakePiston.set(extended ? Value.kForward : Value.kReverse);
    }
}
