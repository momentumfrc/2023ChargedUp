package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX intakeRoller = new VictorSPX(Constants.INTAKE_ROLLER.address);

    public IntakeSubsystem() {
    }

    public void runIntake(double power) {
        intakeRoller.set(VictorSPXControlMode.PercentOutput, power);
    }

    public void idleIntake() {
        runIntake(0);
    }
}
