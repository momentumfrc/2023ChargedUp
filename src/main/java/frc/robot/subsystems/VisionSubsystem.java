package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private UsbCamera lifecam;
    private CvSink inputStream;
    private CvSource outputStream;

    private boolean initialized = false;

    public void setUpIO() {
        // The UsbCamera class causes a segfault when running on Windows, so bail if we're not
        // running on the actual robot
        if(initialized || !RobotBase.isReal()) {
            return;
        }

        lifecam = new UsbCamera("Lifecam", 0);

        lifecam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        inputStream = new CvSink("opencv_Lifecam");
        inputStream.setSource(lifecam);

        outputStream = new CvSource("VisionSubsystem", PixelFormat.kMJPEG, 640, 480, 30);

        initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Get the next available frame from the camera.
     * <p>
     * Note that {@link edu.wpi.first.cscore.CvSink#grabFrame CvSink.grabFrame}
     * blocks until a frame is available, so it is recommended not to
     * call this method from the robot thread.
     * @param frame The Mat in which to store the next frame
     * @return The frame time, or 0 on error
     */
    public synchronized long getFrameFromCamera(Mat frame) {
        return inputStream.grabFrame(frame);
    }

    public synchronized void sendFrameToDashboard(Mat frame) {
        outputStream.putFrame(frame);
    }

}
