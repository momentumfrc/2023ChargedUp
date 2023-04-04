package frc.robot.commands.auto;

import com.momentum4999.utils.PIDTuner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightPipeline;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.TunerUtils;

public class CenterLimelightCrosshairsCommand extends CommandBase {
    private static final double ACCURACY_CUTOFF = 0.15;
    private static final int CROSSHAIR_NOT_FOUND_CUTOFF = 10;
    private static final double SETTINGS_TIME = 0.3;

    private final DriveSubsystem drive;
    private final Limelight limelight;
    private final LimelightPipeline pipeline;

    private final MoPIDF alignmentController = new MoPIDF();
    private final PIDTuner tuner = TunerUtils.forMoPID(alignmentController, "Limelight Fine Alignment");

    private int noCrosshairsCount = 0;

    private Timer setsettingsTimer = new Timer();
    private boolean shouldExit = true;

    public CenterLimelightCrosshairsCommand(DriveSubsystem drive, Limelight limelight, LimelightPipeline pipeline, boolean shouldExit) {
        this.drive = drive;
        this.limelight = limelight;
        this.pipeline = pipeline;

        addRequirements(drive);
        this.shouldExit = shouldExit;
    }

    @Override
    public void initialize() {
        limelight.forceLedsOff(pipeline != LimelightPipeline.REFLECTORS);
        limelight.setPipeline(pipeline);

        setsettingsTimer.restart();
    }

    @Override
    public void execute() {
        if(!setsettingsTimer.hasElapsed(SETTINGS_TIME)) {
            return;
        }
        var maybeCrosshair = limelight.getCrosshair();
        if(maybeCrosshair.isEmpty()) {
            return;
        }
        var crosshair = maybeCrosshair.get();
        double turnRequest = alignmentController.calculate(crosshair.getX(), 0);
        drive.driveCartesian(0, 0, turnRequest);
    }

    @Override
    public boolean isFinished() {
        if(!setsettingsTimer.hasElapsed(SETTINGS_TIME)) {
            return false;
        }
        var maybeCrosshair = limelight.getCrosshair();
        if(maybeCrosshair.isEmpty()) {
            noCrosshairsCount += 1;
            return noCrosshairsCount > CROSSHAIR_NOT_FOUND_CUTOFF;
        } else {
            noCrosshairsCount = 0;
            return Math.abs(maybeCrosshair.get().getX()) < ACCURACY_CUTOFF;
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.forceLedsOff(true);
        limelight.setPipeline(LimelightPipeline.FIDUCIAL);
    }
}
