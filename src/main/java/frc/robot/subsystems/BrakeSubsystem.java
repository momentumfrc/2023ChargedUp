package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BrakeSubsystem extends SubsystemBase {
    private final DoubleSolenoid intakePiston = new DoubleSolenoid(
		PneumaticsModuleType.CTREPCM, Constants.EXTEND_BRAKE, Constants.RETRACT_BRAKE);

    private boolean wasExtended = true;
    private boolean extended = false;

    public BrakeSubsystem() {
    }

    @Override
    public void periodic() {
        if (wasExtended != extended) {
            intakePiston.set(extended ? Value.kForward : Value.kReverse);
        }

        this.wasExtended = this.extended;
    }

    public void setBrakesExtended(boolean extended) {
        this.extended = extended;
    }

    public boolean areBrakesExtended() {
        return this.extended;
    }
}
