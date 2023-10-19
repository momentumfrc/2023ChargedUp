package frc.robot.input;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem.ArmMovementRequest;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.Utils;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;
import frc.robot.utils.MoPrefs.Pref;

public class JoystickControllerInput implements MoInput {

    private final Joystick driveController;
    private final XboxController armController;

    private Pref<Double> driveDeadzone = MoPrefs.driveDeadzone;
    private Pref<Double> driveCurve = MoPrefs.driveCurve;

    public JoystickControllerInput(Constants.HIDPort drivePort, Constants.HIDPort armPort) {
        this.driveController = new Joystick(drivePort.port);
        this.armController = new XboxController(armPort.port);
    }

    private double applyDriveInputTransforms(double value) {
        return Utils.curve(Utils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    private double applyArmInputTransforms(double value) {
        // TODO: determine if we'd rather have a separate deadzone/curve for the arm controller
        return Utils.curve(Utils.deadzone(value, driveDeadzone.get()), driveCurve.get());
    }

    @Override
    public MoveVector getMoveRequest() {
        var mv = new MoveVector(driveController.getRawAxis(1), -1 * driveController.getRawAxis(0));
        mv.applyTransforms(this::applyDriveInputTransforms);

        return mv;
    }

    @Override
    public double getTurnRequest() {
        return -1 * applyDriveInputTransforms(driveController.getRawAxis(2));
    }

    @Override
    public boolean getShouldUseSlowSpeed() {
        return false; //driveController.getRawButton(1);
    }

    @Override
    public boolean getShouldIntake() {
        return armController.getLeftBumper();
    }

    @Override
    public boolean getShouldExhaust() {
        return armController.getRightBumper();
    }

    @Override
    public boolean getShouldAlignCones() {
        return false;
    }

    @Override
    public boolean getShouldAlignCubes() {
        return false;
    }

    @Override
    public ArmMovementRequest getArmMovementRequest() {
        return new ArmMovementRequest(
            applyArmInputTransforms(armController.getLeftY()),
            -1 * applyArmInputTransforms(armController.getRightY())
        );
    }

    @Override
    public Optional<ArmSetpoint> getRequestedArmSetpoint() {
        double pov = armController.getPOV();
        boolean cubes = armController.getXButton();
        boolean cones = armController.getAButton();
        boolean stow = armController.getBButton();
        boolean load = armController.getYButton();

        if (load) {
            return Optional.of(ArmSetpoint.LOADING);
        }

        if(cubes) {
            if(pov == 0) {
                return Optional.of(ArmSetpoint.CUBE_HIGH);
            } else if(pov == 90) {
                return Optional.of(ArmSetpoint.CUBE_PICKUP);
            } else if(pov == 180) {
                return Optional.of(ArmSetpoint.CUBE_LOW);
            } else if(pov == 270) {
                return Optional.of(ArmSetpoint.CUBE_MED);
            }
        }

        if(cones) {
            if(pov == 0) {
                return Optional.of(ArmSetpoint.CONE_HIGH);
            } else if(pov == 90) {
                return Optional.of(ArmSetpoint.CONE_PICKUP);
            } else if(pov == 180) {
                return Optional.of(ArmSetpoint.CONE_LOW);
            } else if(pov == 270) {
                return Optional.of(ArmSetpoint.CONE_MED);
            }
        }

        if(stow) {
            return Optional.of(ArmSetpoint.STOW);
        }

        return Optional.empty();
    }

    @Override
    public boolean getSaveArmSetpoint() {
        return armController.getStartButton();
    }

    @Override
    public boolean getShouldBrake() {
        return false;
    }

    public boolean getReZeroArms() {
        return armController.getBackButtonPressed();
    }

    @Override
    public boolean getShouldBalance() {
        return false; //driveController.getRawButton(7);
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getRawButton(11);
    }

    @Override
    public boolean getShouldDriveAligned() {
        return false;
    }

}
