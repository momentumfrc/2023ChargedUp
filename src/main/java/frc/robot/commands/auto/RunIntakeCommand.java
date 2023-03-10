package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.MoPrefs;

public class RunIntakeCommand extends CommandBase {
    /**
     * How long (in seconds) to run the intake.
     */
    private static final double INTAKE_TIME = 0.5;

    public static enum IntakeDirection {
        CONE_INTAKE(1),
        CONE_EXHAUST(-1),
        CUBE_INTAKE(-1),
        CUBE_EXHAUST(1);

        public final double direction;
        private IntakeDirection(double direction) {
            this.direction = direction;
        }
    }

    private final IntakeSubsystem intake;
    private final IntakeDirection direction;
    private final Timer runTimer = new Timer();

    public RunIntakeCommand(IntakeSubsystem intake, IntakeDirection direction) {
        this.intake = intake;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        runTimer.restart();
    }

    @Override
    public void execute() {
        double power = MoPrefs.intakeSpeed.get() * direction.direction;
        intake.runIntake(power);
    }

    @Override
    public void end(boolean interrupted) {
        runTimer.stop();
        intake.idleIntake();
    }

    @Override
    public boolean isFinished() {
        return runTimer.hasElapsed(INTAKE_TIME);
    }
}
