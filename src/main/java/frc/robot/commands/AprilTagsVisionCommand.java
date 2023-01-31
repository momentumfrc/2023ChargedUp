package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagsVisionCommand extends DefaultVisionCommand {

    private final AprilTagDetector detector;
    private final AprilTagPoseEstimator poseEstimator;
    private AprilTagFieldLayout fieldLayout;

    private final Consumer<Pose3d> poseConsumer;

    private Mat bwFrame = new Mat();

    public AprilTagsVisionCommand(VisionSubsystem visionSubsystem, Consumer<Pose3d> poseConsumer) {
        super(visionSubsystem);
        this.poseConsumer = poseConsumer;

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

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(IOException e) {
            e.printStackTrace();
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

    /**
     * Returns a single estimated pose for a list of possible poses by averaging them all together.
     * @param poses The possible poses
     * @return The average of the input poses.
     */
    private Optional<Pose3d> coalesceEstimatedPoses(ArrayList<Pose3d> poses) {
        if(poses.size() == 0) {
            Optional.empty();
        }
        if(poses.size() == 1) {
            return Optional.of(poses.get(0));
        }

        Translation3d translation = new Translation3d();
        Rotation3d rotation = new Rotation3d();
        for(Pose3d pose : poses) {
            translation.plus(pose.getTranslation());
            rotation.plus(pose.getRotation());
        }

        translation.div(poses.size());
        rotation.div(poses.size());

        return Optional.of(new Pose3d(translation, rotation));
    }

    @Override
    public void processFrame(Mat capturedFrame) {
        Imgproc.cvtColor(capturedFrame, bwFrame, Imgproc.COLOR_RGB2GRAY);

        AprilTagDetection[] detections = detector.detect(bwFrame);
        ArrayList<Pose3d> estimatedPoses = new ArrayList<Pose3d>(detections.length);
        for(int i = 0; i < detections.length; ++i) {
            AprilTagDetection detection = detections[i];
            Transform3d estimatedTransform = poseEstimator.estimate(detection);
            drawDetection(detection, estimatedTransform, capturedFrame);

            if(fieldLayout != null) {
                var maybeTagPose = fieldLayout.getTagPose(detection.getId());
                if(maybeTagPose.isPresent()) {
                    estimatedPoses.add(maybeTagPose.get().transformBy(estimatedTransform));
                }
            }
        }

        var coalescedPose = coalesceEstimatedPoses(estimatedPoses);
        if(coalescedPose.isPresent()) {
            poseConsumer.accept(coalescedPose.get());
        }
    }

}
