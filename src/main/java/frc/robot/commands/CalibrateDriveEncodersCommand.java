package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.CorrectionFactorCalculator;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.MoPrefs.Pref;

public class CalibrateDriveEncodersCommand extends CommandBase {
    private static final double CALIBRATE_SPEED = 0.03;
    public static final int N_RUNS = 5;

    private class Calibrator {
        private SwerveModule module;
        private Pref<Double> encoderScale;
        private CorrectionFactorCalculator calculator;

        private double[] calculatedValues = new double[N_RUNS];

        private int runs = 0;

        public Calibrator(SwerveModule module, Pref<Double> encoderScale) {
            this.module = module;
            this.encoderScale = encoderScale;

            this.calculator = new CorrectionFactorCalculator(
                module.absoluteEncoder::getPosition,
                module.turnMotor.getEncoder()::getPosition
            );
        }

        public void initialize() {
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

                calculator.start();
            }

            module.directDrive(CALIBRATE_SPEED, 0);
            calculator.recordDataPoint();
        }

        public boolean isFinished() {
            return runs >= N_RUNS;
        }

        public void end() {
            double mean = 0;
            for(double value : calculatedValues) {
                mean += value;
            }

            mean /= N_RUNS;

            encoderScale.set(encoderScale.get() * mean);
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
        if(interrupted) {
            return;
        }
        for(Calibrator c : calibrators) {
            c.end();
        }
    }
}
