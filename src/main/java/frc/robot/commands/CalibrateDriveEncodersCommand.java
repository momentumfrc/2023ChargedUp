package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.CorrectionFactorCalculator;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoUtils;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.MoPrefs.Pref;

public class CalibrateDriveEncodersCommand extends CommandBase {
    private static final double CALIBRATE_SPEED = 0.03;
    private static final double CALIBRATE_STEP = 0.05;
    private static final int COOLDOWN_STEPS = 3;

    public static final int N_RUNS = 5;

    /**
     * This class takes a single swerve module, runs 5 calibration cycles, and uses the average
     * of the results to adjust the relative encoder scale.
     * <p>
     * During a calibration cycle, the motor is used to spin the encoders through most of a single
     * revolution, during which the CorrectionFactorCalculator notes the values reported by the
     * absolute and relative encoders. Then, at the end of the cycle, the calculator uses the noted
     * values to calculate the correction factor for the relative encoder scale.
     * <p>
     * Note that during a calibration cycle, the motor does not spin continuously. Instead, the
     * motor is shut off and the mechanism allowed to come to a halt before the encoder values are
     * saved. This helps keep electromagnetic interference from the motor from affecting the
     * the absolute encoder, which is read via an analog voltage. Also, it eliminates any error
     * that could be introduced by the lag time between reading the absolute and relative
     * encoders--since the mechanism isn't be moving, it doesn't matter if the two encoders are not
     * read at precisely the same time. This stepwise motion can be adjusted using the
     * CALIBRATE_STEP and COOLDOWN_STEPS variables.
     * <p>
     * Also note that this class, while written in a similar fashion to a command, is not itself a
     * command. This is for two reasons. First, the each individual swerve module is not its own
     * subsystem--they're all part of the drive subsystem--so we can't use addRequirements. Second,
     * we want to enable some default behavior (no motion/zero output) once isFinished returns true.
     * While we could probably make it work using some combination of ParallelCommandGroup and
     * RunCommand, I find this easier to read and understand.
     */
    private static class Calibrator {
        private SwerveModule module;
        private Pref<Double> encoderScale;
        private CorrectionFactorCalculator calculator;

        private double[] calculatedValues = new double[N_RUNS];

        private int runs = 0;
        private double next_step;
        private int cooldown;

        public Calibrator(SwerveModule module, Pref<Double> encoderScale) {
            this.module = module;
            this.encoderScale = encoderScale;

            this.calculator = new CorrectionFactorCalculator(
                module.absoluteEncoder::getPosition,
                module.turnMotor.getEncoder()::getPosition
            );
        }

        public void initialize() {
            next_step = CALIBRATE_STEP;
            cooldown = COOLDOWN_STEPS;
            calculator.start();
        }

        public void execute() {
            if(isFinished()) {
                module.directDrive(0, 0);
                return;
            }

            if(calculator.isFinished()) {
                calculatedValues[runs] = calculator.calculateCorrectionFactor();
                runs += 1;

                if(isFinished()) {
                    module.directDrive(0, 0);
                    return;
                }

                initialize();
            }

            // We need to use getAbsRad() so that the value is relative to the absolute zero
            // from when the calculator was started.
            if(MoUtils.radToRot(calculator.getAbsRad()) < next_step) {
                module.directDrive(CALIBRATE_SPEED, 0);
            } else {
                module.directDrive(0, 0);

                if(cooldown > 0) {
                    cooldown -= 1;
                } else {
                    calculator.recordDataPoint();
                    next_step += CALIBRATE_STEP;
                    cooldown = COOLDOWN_STEPS;
                }
            }
        }

        public boolean isFinished() {
            return runs >= N_RUNS;
        }

        public void end(boolean interrupted) {
            if(!interrupted) {
                double mean = 0;
                for(double value : calculatedValues) {
                    mean += value;
                }

                mean /= N_RUNS;

                encoderScale.set(encoderScale.get() * mean);
            }

        }
    }

    private final Calibrator[] calibrators;

    public CalibrateDriveEncodersCommand(DriveSubsystem drive) {
        addRequirements(drive);

        calibrators = new Calibrator[] {
            new Calibrator(drive.frontLeft, MoPrefs.flScale),
            new Calibrator(drive.frontRight, MoPrefs.frScale),
            new Calibrator(drive.rearLeft, MoPrefs.rlScale),
            new Calibrator(drive.rearRight, MoPrefs.rrScale)
        };
    }

    @Override
    public void initialize() {
        for(Calibrator c : calibrators) {
            c.initialize();
        }
    }

    @Override
    public void execute() {
        for(Calibrator c : calibrators) {
            c.execute();
        }
    }

    @Override
    public boolean isFinished() {
        for(Calibrator c : calibrators) {
            if(!c.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        for(Calibrator c : calibrators) {
            c.end(interrupted);
        }
    }
}
