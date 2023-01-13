package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MoShuffleboard {
    private static MoShuffleboard instance;

    public static MoShuffleboard getInstance() {
        if(instance == null) {
            instance = new MoShuffleboard();
        }
        return instance;
    }

    public final ShuffleboardTab matchTab;
    public final ShuffleboardTab settingsTab;


    private MoShuffleboard() {
        matchTab = Shuffleboard.getTab("match");
        settingsTab = Shuffleboard.getTab("Settings");
    }

}
