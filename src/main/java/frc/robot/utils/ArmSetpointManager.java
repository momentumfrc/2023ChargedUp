package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Optional;

import org.ini4j.Ini;

import frc.robot.subsystems.ArmSubsystem.ArmPosition;

public class ArmSetpointManager {
    private static class DataStore {
        private static HashMap<File, DataStore> instances = new HashMap<>();
        public static DataStore getInstance(File file) {
            if(!instances.containsKey(file)) {
                instances.put(file, new DataStore(file));
            }
            return instances.get(file);
        }

        private final File file;
        private final Ini ini;

        private DataStore(File file) {
            this.file = file;
            ini = new Ini();
            if(file.isFile()) {
                try {
                    ini.load(file);
                } catch(IOException e) {
                    e.printStackTrace();
                }
            }
        }

        public void putValue(String section, String property, double value) {
            ini.put(section, property, value);
        }

        public Optional<Double> getValue(String section, String property) {
            if(ini.containsKey(section) && ini.get(section).containsKey(property)) {
                return Optional.of(ini.get(section, property, Double.class));
            } else {
                return Optional.empty();
            }
        }

        public void save() {
            try {
                ini.store(file);
            } catch(IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static enum ArmSetpoint {
        CUBE_HIGH,
        CUBE_MED,
        CUBE_LOW,
        CUBE_PICKUP,
        CONE_HIGH,
        CONE_MED,
        CONE_LOW,
        CONE_PICKUP,
        STOW
    };

    private static ArmSetpointManager instance;

    public static ArmSetpointManager getInstance() {
        if(instance == null) {
            instance = new ArmSetpointManager();
        }
        return instance;
    }

    private static class ArmSetpointEntry {
        private String sectionName;
        private DataStore store;

        private ArmPosition position;

        public ArmSetpointEntry(String sectionName, DataStore store) {
            this.sectionName = sectionName;
            this.store = store;

            double shoulder = store.getValue(sectionName, "shoulder").orElse(0.0);
            double wrist = store.getValue(sectionName, "wrist").orElse(0.0);

            this.position = new ArmPosition(shoulder, wrist);
        }

        public ArmPosition getValue() {
            return position;
        }

        public void setValue(ArmPosition position) {
            if(position.equals(this.position))
                return;
            this.position = position;

            store.putValue(sectionName, "shoulder", position.shoulderRotations);
            store.putValue(sectionName, "wrist", position.wristRotations);
            store.save();
        }
    }

    private DataStore store = DataStore.getInstance(new File("/home/lvuser/arm_setpoints.ini"));
    private EnumMap<ArmSetpoint, ArmSetpointEntry> entries = new EnumMap<>(ArmSetpoint.class);

    private void initSetpoint(ArmSetpoint setpoint) {
        entries.put(setpoint, new ArmSetpointEntry(setpoint.name(), store));
    }

    private ArmSetpointManager() {
        for(ArmSetpoint setpoint : ArmSetpoint.values()) {
            initSetpoint(setpoint);
        }
    }

    public ArmPosition getSetpoint(ArmSetpoint setpoint) {
        return entries.get(setpoint).getValue();
    }

    public void setSetpoint(ArmSetpoint setpoint, ArmPosition position) {
        entries.get(setpoint).setValue(position);
    }

}
