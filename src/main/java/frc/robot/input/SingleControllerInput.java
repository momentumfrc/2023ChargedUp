package frc.robot.input;

import static com.momentum4999.utils.Utils.*;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.Pref;

public class SingleControllerInput implements MoInput {
    private final XboxController controller;

    private Pref<Double> deadzone = MoPrefs.driveDeadzone;
    private Pref<Double> curve = MoPrefs.driveCurve;

    private Map<String, ArmPositionRequest> armPositions = ArmPositionRequest.loadPositionsFromFile();
    private ArmPositionRequest lastRequestedPosition;

    public SingleControllerInput(Constants.HIDPort port) {
        this.controller = new XboxController(port.port);

        lastRequestedPosition = armPositions.get("stowed");
        if(lastRequestedPosition == null) {
            lastRequestedPosition = new ArmPositionRequest(0, 0);
        }
    }

    private double applyInputTransforms(double input) {
        return curve(deadzone(input, deadzone.get()), curve.get());
    }

    @Override
    public double getForwardSpeedRequest() {
        return applyInputTransforms(controller.getLeftY());
    }

    @Override
    public double getLeftSpeedRequest() {
        return -1 * applyInputTransforms(controller.getLeftX());
    }

    @Override
    public double getTurnRequest() {
        return -1 * applyInputTransforms(controller.getRightX());
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return controller.getLeftStickButton();
    }

    @Override
    public double getDirectShoulderRequest() {
        return applyInputTransforms(this.controller.getRightTriggerAxis() - this.controller.getLeftTriggerAxis());
    }

    @Override
    public double getDirectWristRequest() {
        return applyInputTransforms(this.controller.getRightY());
    }

    @Override
    public boolean getShouldIntake() {
        return this.controller.getLeftBumper();
    }

    @Override
    public boolean getShouldExhaust() {
        return this.controller.getRightBumper();
    }

    @Override
    public ArmPositionRequest getArmPositionRequest() {
        double pov = this.controller.getPOV();
        boolean cubes = this.controller.getXButton();
        boolean cones = this.controller.getAButton();
        ArmPositionRequest retval = null;
        if(pov == 90) {
            retval = armPositions.get("stowed");
        } else if(pov == 180) {
            if(cubes || cones) {
                retval = armPositions.get("hybrid_node");
            } else {
                retval = armPositions.get("ground_pickup");
            }
        } else if(pov == 270) {
            if(cubes) {
                retval = armPositions.get("med_cube_node");
            } else if(cones) {
                retval = armPositions.get("med_cone_node");
            }
        } else if(pov == 0) {
            if(cubes) {
                retval = armPositions.get("high_cube_node");
            } else if(cones) {
                retval = armPositions.get("high_cone_node");
            }
        }

        if(retval == null) {
            retval = lastRequestedPosition;
        } else {
            lastRequestedPosition = retval;
        }
        return retval;
    }
}
