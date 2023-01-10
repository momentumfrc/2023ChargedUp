package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MoShuffleboard;

public class VisionSubsystem extends SubsystemBase {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private final UsbCamera lifecam;
    private final CvSink inputStream;
    private final CvSource outputStream;
    private final MjpegServer mjpegServer;

    private AprilTagDetector detector;
    private AprilTagPoseEstimator poseEstimator;

    private GenericSubscriber shouldDetectAprilTags;

    private Thread visionThread;

    private Mat currFrame = new Mat();
    private Mat outFrame = new Mat();
    private Mat bwFrame = new Mat();

    public VisionSubsystem() {
        lifecam = new UsbCamera("Lifecam", 0);

        lifecam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        inputStream = new CvSink("opencv_Lifecam");
        inputStream.setSource(lifecam);

        outputStream = new CvSource("VisionSubsystem", PixelFormat.kMJPEG, 320, 240, 30);
        mjpegServer = new MjpegServer("serve_VisionSubsystem", 1181);
        mjpegServer.setSource(outputStream);

        shouldDetectAprilTags = MoShuffleboard.getInstance().detectAprilTagsSwitch.getEntry();

        AprilTagDetector.Config detectorConfig = new AprilTagDetector.Config();
        detectorConfig.numThreads = 3;
        detectorConfig.quadDecimate = 3;
        detectorConfig.quadSigma = 0;

        detector = new AprilTagDetector();
        detector.setConfig(detectorConfig);

        detector.addFamily("tag16h5", 0);

        AprilTagPoseEstimator.Config poseEstimatorConfig = new AprilTagPoseEstimator.Config(
            0.1524,
            672.5873577443733,
            673.3134462534064,
            333.5730281629913,
            226.2345964911072
        );
        poseEstimator = new AprilTagPoseEstimator(poseEstimatorConfig);

        visionThread = new Thread(() -> {
            while(!Thread.interrupted()) {
                long frameTime = inputStream.grabFrame(currFrame);
                if(frameTime > 0) {
                    if(shouldDetectAprilTags.getBoolean(true)) {
                        Imgproc.cvtColor(currFrame, bwFrame, Imgproc.COLOR_RGB2GRAY);

                        AprilTagDetection[] detections = detector.detect(bwFrame);
                        for(AprilTagDetection detection : detections) {
                            Transform3d estimatedPose = poseEstimator.estimate(detection);
                            drawDetection(detection, estimatedPose, currFrame);
                        }
                    }
                    Imgproc.resize(currFrame, outFrame, new Size(320, 240));
                    outputStream.putFrame(outFrame);
                }
            }
        });
        if(RobotBase.isReal()) {
            // Something in the visionThread causes a segfault when running on Windows. I'm guessing there's something wrong
            // with the windows builds of wpilib native libraries. The solution is to only run the vision code when running
            // on the actual robot.
            visionThread.start();
        }
    }

    private void drawDetection(AprilTagDetection detection, Transform3d estimatedTransform, Mat outMat) {
        final double fontSize = 1.1;
        final int thickness = 1;

        for(int i = 0; i < 4; ++i) {
            Point prev = new Point(detection.getCornerX((i + 3) % 4), detection.getCornerY((i + 3) % 4));
            Point curr = new Point(detection.getCornerX(i), detection.getCornerY(i));

            Imgproc.line(outMat, prev, curr, new Scalar(255, 0, 255), thickness, 1);
        }

        String label = Integer.toString(detection.getId());

        Point textPoint = new Point(detection.getCornerX(3), detection.getCornerY(3)-5);
        Imgproc.putText(outMat, label, textPoint, Imgproc.FONT_HERSHEY_PLAIN, fontSize, new Scalar(255, 0, 255), thickness);

        String translationTxt = String.format("(x:%.2f, y:%.2f, z:%.2f)", estimatedTransform.getX(), estimatedTransform.getY(), estimatedTransform.getZ());
        textPoint.x += 20;
        Imgproc.putText(outMat, translationTxt, textPoint, Imgproc.FONT_HERSHEY_PLAIN, fontSize, new Scalar(0, 255, 0), thickness);

        Rotation3d rot = estimatedTransform.getRotation();
        String rotTxt = String.format("(r:%.2f, p:%.2f, y:%.2f)", rot.getX(), rot.getY(), rot.getZ());
        textPoint.y += 20;
        Imgproc.putText(outMat, rotTxt, textPoint, Imgproc.FONT_HERSHEY_PLAIN, fontSize, new Scalar(0, 128, 255), thickness);
    }
}
