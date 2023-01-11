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
import edu.wpi.first.cscore.VideoException;
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

    private UsbCamera lifecam;
    private CvSink inputStream;
    private CvSource outputStream;
    private MjpegServer mjpegServer;

    private AprilTagDetector detector;
    private AprilTagPoseEstimator poseEstimator;

    private GenericSubscriber shouldDetectAprilTags;

    private Thread visionThread;

    private Mat currFrame = new Mat();
    private Mat outFrame = new Mat();
    private Mat bwFrame = new Mat();

    private boolean initialized = false;

    public boolean init() {
        try {
            lifecam = new UsbCamera("Lifecam", 0);

            lifecam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

            inputStream = new CvSink("opencv_Lifecam");
            inputStream.setSource(lifecam);

            outputStream = new CvSource("VisionSubsystem", PixelFormat.kMJPEG, 320, 240, 30);
            // mjpegServer = new MjpegServer("serve_VisionSubsystem", 1181);
            // mjpegServer.setSource(outputStream);

            MoShuffleboard.getInstance().matchTab.add(outputStream);

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
                while(initialized && !Thread.interrupted() ) {
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

            visionThread.start();
            initialized = true;
        } catch(VideoException e) {
            // If the visionSubsystem fails to initialize (for example, if the webcam is not plugged
            // in), we shouldn't crash the whole robot. We should still be able to at least drive.
            System.out.println("Exception when initializing the VisionSubsystem");
            e.printStackTrace();
            initialized = false;
        }
        return initialized;
    }

    public VisionSubsystem() {}

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
