package frc.robot.input;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class ArmPositionRequest {
    private static final String ARM_POSITIONS_FILE = "armPositions.json";

    public final double wristRotations;
    public final double shoulderRotations;

    public ArmPositionRequest(double wristRotations, double shoulderRotations) {
        this.wristRotations = wristRotations;
        this.shoulderRotations = shoulderRotations;
    }

    @Override
    public boolean equals(Object other) {
        if(other instanceof ArmPositionRequest) {
            ArmPositionRequest aprOther = (ArmPositionRequest) other;
            return wristRotations == aprOther.wristRotations
                && shoulderRotations == aprOther.shoulderRotations;
        }
        return false;
    }

    private static double parseNumeric(Object input) {
        if(input instanceof Long) {
            long value = (Long) input;
            return value;
        }
        return (Double) input;
    }

    @Override
    public String toString() {
        return String.format("ArmPositionRequest(shoulder=%.2f, wrist=%.2f)", shoulderRotations, wristRotations);
    }

    public static Map<String, ArmPositionRequest> loadPositionsFromFile() {
        HashMap<String, ArmPositionRequest> output = new HashMap<>();
        try (BufferedReader br = new BufferedReader(
            new FileReader(new File(Filesystem.getDeployDirectory(), ARM_POSITIONS_FILE))
        )) {
            JSONObject json = (JSONObject) new JSONParser().parse(br);
            for(Object untypedEntry : json.entrySet()) {
                Entry<String, JSONObject> entry = (Entry<String, JSONObject>) untypedEntry;
                ArmPositionRequest request = new ArmPositionRequest(
                    parseNumeric(entry.getValue().get("wrist")),
                    parseNumeric(entry.getValue().get("shoulder"))
                );
                output.put(entry.getKey(), request);
            }
        } catch(IOException|ParseException|ClassCastException|NullPointerException e) {
            DriverStation.reportError("Unable to load arm positions: " + e.getMessage(), e.getStackTrace());
            return Map.of();
        }
        return output;
    }

}
