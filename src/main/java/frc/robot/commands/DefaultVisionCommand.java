package frc.robot.commands;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The DefaultVisionCommand just grabs frames from the visionSubsystem at 640x480,
 * resizes them to 320x240, and returns them to the visionSubsystem to be streamed to the
 * Shuffleboard.
 * <p>
 * Subclasses may override {@link #processFrame(Mat)} to perform additional processing on the frames
 * before they are resized or returned to the visionSubsystem.
 */
public class DefaultVisionCommand extends CommandBase {
    private final VisionSubsystem visionSubsystem;

    private Thread visionThread;

    private Mat capturedFrame = new Mat();
    private Mat outputFrame = new Mat();

    public DefaultVisionCommand(VisionSubsystem visionSubsystem) {
        addRequirements(visionSubsystem);
        this.visionSubsystem = visionSubsystem;
    }

    protected void processFrame(Mat capturedFrame) {}

    @Override
    public void initialize() {
        // Vision processing happens in a separate thread because many vision processing operations
        // are synchronous and take a long time, which would cause loop overruns
        visionThread = new Thread(() -> {
            while(visionSubsystem.isInitialized() && !Thread.interrupted()) {
                long frameTime = visionSubsystem.getFrameFromCamera(capturedFrame);
                if(frameTime > 0) {
                    processFrame(capturedFrame);
                    Imgproc.resize(capturedFrame, outputFrame, new Size(320, 240));
                    visionSubsystem.sendFrameToDashboard(outputFrame);
                }
            }
        });

        try {
            visionSubsystem.setUpIO();
            visionThread.start();
        } catch(VideoException e) {
            DriverStation.reportError("Exception when initializing the DetectAprilTagsCommand", e.getStackTrace());
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        visionThread.interrupt();
        try {
            visionThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
