package frc.robot.commands.auto;

import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.motune.PIDTuner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPIDF;
import frc.robot.utils.TunerUtils;
import frc.robot.utils.Utils;

/**
 * A simple bang-bang controller to automatically level the scale.
 */
public class BalanceScaleCommand extends CommandBase {
    private static final double MAX_MOVE_SPEED = 0.1;
    private static final double LEVEL_DEFINITION = 1.5; // degrees

    private final MoPIDF balanceScalePid = new MoPIDF();

    private final DriveSubsystem drive;
    private final AHRS navx;

    public BalanceScaleCommand(DriveSubsystem drive, AHRS navx) {
        this.drive = drive;
        this.navx = navx;

        TunerUtils.forMoPID(balanceScalePid, "Balance Scale");

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double pitch = navx.getPitch();
        if(pitch < -LEVEL_DEFINITION) {
            pitch += LEVEL_DEFINITION;
        } else if(pitch > LEVEL_DEFINITION) {
            pitch -= LEVEL_DEFINITION;
        } else {
            pitch = 0;
        }

        double moveRequest = -1 * balanceScalePid.calculate(pitch, 0);
        moveRequest = Utils.clip(moveRequest, -MAX_MOVE_SPEED, MAX_MOVE_SPEED);

        drive.driveCartesian(moveRequest, 0, 0);
    }

}
