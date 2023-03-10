package frc.robot.utils;

import java.util.EnumMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

public class ArmSetpointManager {
    private static final String TABLE_NAME = "ArmSetpoints";

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
        public final NetworkTableEntry shoulderEntry;
        public final NetworkTableEntry wristEntry;

        public ArmSetpointEntry(NetworkTableEntry shoulderEntry, NetworkTableEntry wristEntry) {
            this.shoulderEntry = shoulderEntry;
            this.wristEntry = wristEntry;
        }

        public ArmPosition getValue() {
            return new ArmPosition(
                shoulderEntry.getDouble(0),
                wristEntry.getDouble(0)
            );
        }

        public void setValue(ArmPosition position) {
            shoulderEntry.setDouble(position.shoulderRotations);
            wristEntry.setDouble(position.wristRotations);
        }
    }

    private NetworkTable table;

    private EnumMap<ArmSetpoint, ArmSetpointEntry> entries = new EnumMap<>(ArmSetpoint.class);

    private void initSetpoint(ArmSetpoint setpoint) {
        NetworkTableEntry shoulderEntry = table.getEntry(setpoint.name() + "_shoulder");
        NetworkTableEntry wristEntry = table.getEntry(setpoint.name() + "_wrist");
        shoulderEntry.setDefaultDouble(0);
        wristEntry.setDefaultDouble(0);
        shoulderEntry.setPersistent();
        wristEntry.setPersistent();
        entries.put(setpoint, new ArmSetpointEntry(shoulderEntry, wristEntry));
    }

    private ArmSetpointManager() {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);

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
