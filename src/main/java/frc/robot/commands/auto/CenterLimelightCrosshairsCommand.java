package frc.robot.commands.auto;

import com.momentum4999.utils.PIDTuner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightPipeline;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.TunerUtils;

public class CenterLimelightCrosshairsCommand extends CommandBase {
    private static final int CROSSHAIR_NOT_FOUND_CUTOFF = 10;

    private final DriveSubsystem drive;
    private final Limelight limelight;
    private final LimelightPipeline pipeline;

    private final MoPIDF alignmentController = new MoPIDF();
    private final PIDTuner tuner = TunerUtils.forMoPID(alignmentController, "Limelight Fine Alignment");

    private int noCrosshairsCount = 0;

    public CenterLimelightCrosshairsCommand(DriveSubsystem drive, Limelight limelight, LimelightPipeline pipeline) {
        this.drive = drive;
        this.limelight = limelight;
        this.pipeline = pipeline;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        limelight.forceLedsOff(pipeline != LimelightPipeline.REFLECTORS);
        limelight.setPipeline(pipeline);
    }

    @Override
    public void execute() {
        var maybeCrosshair = limelight.getCrosshair();
        if(maybeCrosshair.isEmpty()) {
            return;
        }
        var crosshair = maybeCrosshair.get();
        double leftRequest = alignmentController.calculate(crosshair.getX(), 0);
        drive.driveCartesian(0, leftRequest, 0);
    }

    @Override
    public boolean isFinished() {
        var maybeCrosshair = limelight.getCrosshair();
        if(maybeCrosshair.isEmpty()) {
            noCrosshairsCount += 1;
            return noCrosshairsCount > CROSSHAIR_NOT_FOUND_CUTOFF;
        } else {
            noCrosshairsCount = 0;
            return Limelight.areCrosshairsZeroed(maybeCrosshair.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelight.forceLedsOff(true);
        limelight.setPipeline(LimelightPipeline.FIDUCIAL);
    }
}
