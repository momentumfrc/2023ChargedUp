package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class MoShuffleboard {
    private static MoShuffleboard instance;

    public static MoShuffleboard getInstance() {
        if(instance == null) {
            instance = new MoShuffleboard();
        }
        return instance;
    }

    public final SimpleWidget detectAprilTagsSwitch;


    private MoShuffleboard() {
        detectAprilTagsSwitch = Shuffleboard.getTab("match").add("Detect AprilTags", true).withWidget(BuiltInWidgets.kToggleSwitch);
    }

}
