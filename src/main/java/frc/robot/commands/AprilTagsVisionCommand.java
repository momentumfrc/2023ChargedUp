package frc.robot.commands;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagsVisionCommand extends DefaultVisionCommand {

    private final AprilTagDetector detector;
    private final AprilTagPoseEstimator poseEstimator;

    private Mat bwFrame = new Mat();

    public AprilTagsVisionCommand(VisionSubsystem visionSubsystem) {
        super(visionSubsystem);

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

    @Override
    public void processFrame(Mat capturedFrame) {
        Imgproc.cvtColor(capturedFrame, bwFrame, Imgproc.COLOR_RGB2GRAY);

        AprilTagDetection[] detections = detector.detect(bwFrame);
        for(AprilTagDetection detection : detections) {
            Transform3d estimatedPose = poseEstimator.estimate(detection);
            drawDetection(detection, estimatedPose, capturedFrame);
        }
    }

}
