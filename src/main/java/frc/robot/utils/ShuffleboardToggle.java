package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShuffleboardToggle implements BooleanSupplier {
    private GenericEntry entry;
    private boolean defaultValue;

    public ShuffleboardToggle(ShuffleboardTab tab, String name, boolean startValue) {
        defaultValue = startValue;
        entry = tab.add(name, startValue).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    @Override
    public boolean getAsBoolean() {
        return entry.getBoolean(defaultValue);
    }

    public Trigger getTrigger() {
        return new Trigger(this);
    }

}
