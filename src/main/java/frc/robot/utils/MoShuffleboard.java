package frc.robot.utils;

import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
    public final ShuffleboardTab autoTab;
    public final Field2d field;

    private MoShuffleboard() {
        matchTab = Shuffleboard.getTab("match");
        settingsTab = Shuffleboard.getTab("Settings");
        autoTab = Shuffleboard.getTab("Auto");

        field = new Field2d();
        matchTab.add(field);
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert) {
        boolean setDefault = true;
        var chooser = new SendableChooser<T>();
        for(T entry : toConvert.getEnumConstants()) {
            if(setDefault) {
                chooser.setDefaultOption(entry.name(), entry);
                setDefault = false;
            } else {
                chooser.addOption(entry.name(), entry);
            }
        }
        return chooser;
    }

    public static <T extends Nameable> SendableChooser<T> listToChooser(List<T> toConvert) {
        boolean setDefault = true;
        var chooser = new SendableChooser<T>();
        for(T entry : toConvert) {
            if(setDefault) {
                chooser.setDefaultOption(entry.getName(), entry);
                setDefault = false;
            } else {
                chooser.addOption(entry.getName(), entry);
            }
        }
        return chooser;
    }
}
