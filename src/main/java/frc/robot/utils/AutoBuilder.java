package frc.robot.utils;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.BalanceScaleCommand;
import frc.robot.commands.auto.CenterLimelightCrosshairsCommand;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightPipeline;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PositioningSubsystem;

public class AutoBuilder {
    public enum MobilityPosition {
        MOB_L,
        MOB_R
    }

    public enum PieceType {
        NONE,
        CUBE,
        CONE
    }

    public enum StartingPosition {
        FLOAT_L(PieceType.NONE, MobilityPosition.MOB_L),
        FLOAT_R(PieceType.NONE, MobilityPosition.MOB_R),
        GRID_LL(PieceType.CONE, MobilityPosition.MOB_L),
        GRID_LC(PieceType.CUBE, MobilityPosition.MOB_L),
        GRID_LR(PieceType.CONE, MobilityPosition.MOB_L),
        GRID_CL(PieceType.CONE, MobilityPosition.MOB_L),
        GRID_CC(PieceType.CUBE, MobilityPosition.MOB_L),
        GRID_CR(PieceType.CONE, MobilityPosition.MOB_R),
        GRID_RL(PieceType.CONE, MobilityPosition.MOB_R),
        GRID_RC(PieceType.CUBE, MobilityPosition.MOB_R),
        GRID_RR(PieceType.CONE, MobilityPosition.MOB_R);

        public PieceType pieceType;
        public final MobilityPosition mobPos;
        private StartingPosition(PieceType pieceType, MobilityPosition mobPos) {
            this.pieceType = pieceType;
            this.mobPos = mobPos;
        }
    }

    public enum ScoreLevel {
        LEVEL_NONE,
        LEVEL_HYBRID,
        LEVEL_LOW,
        LEVEL_HIGH
    };

    private SendableChooser<StartingPosition> startPosChooser;
    private SendableChooser<ScoreLevel> scoreLevelChooser;
    private SendableChooser<PieceType> heldPieceType;
    private GenericEntry masterAutoSwitch;
    private GenericEntry limelightAlignment;
    private GenericEntry chargeStationDocking;
    private GenericEntry chargeStationEngagement;

    public void initShuffleboard() {
        startPosChooser = MoShuffleboard.enumToChooser(StartingPosition.class);
        scoreLevelChooser = MoShuffleboard.enumToChooser(ScoreLevel.class);
        heldPieceType = MoShuffleboard.enumToChooser(PieceType.class);

        ShuffleboardTab tab = MoShuffleboard.getInstance().autoTab;
        masterAutoSwitch = tab.add("Master Auto Switch", true)
            .withPosition(0, 0)
            .withSize(4, 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        tab.add("Starting Position", startPosChooser)
            .withPosition(0, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kComboBoxChooser);

        tab.add("Scoring Level", scoreLevelChooser)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kComboBoxChooser);

        tab.add("Held Piece Type", heldPieceType)
            .withPosition(0, 3)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kComboBoxChooser);

        limelightAlignment = tab.add("Limelight Alignment", false)
            .withPosition(2, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        chargeStationDocking = tab.add("Dock Charge Station", false)
            .withPosition(2, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

        chargeStationEngagement = tab.add("Engage Charge Station", false)
            .withPosition(2, 3)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    }

    public Command buildAutoCommand(DriveSubsystem drive, PositioningSubsystem pos, AHRS navx) {
        boolean masterAutoSwitch = this.masterAutoSwitch.getBoolean(false);
        StartingPosition startPos = this.startPosChooser.getSelected();
        ScoreLevel selectedScoreLevel = this.scoreLevelChooser.getSelected();
        PieceType heldGamePiece = this.heldPieceType.getSelected();
        boolean limelightAlignment = this.limelightAlignment.getBoolean(false);
        boolean chargeStationDocking = this.chargeStationDocking.getBoolean(false);
        boolean chargeStationEngagement = this.chargeStationEngagement.getBoolean(false);

        // 1
        if(!masterAutoSwitch) {
            return new RunCommand(() -> {});
        }

        ArrayList<Command> commands = new ArrayList<>();

        // 2
        if(startPos.pieceType != PieceType.NONE && selectedScoreLevel != ScoreLevel.LEVEL_NONE) {
            // 2.a
            if(limelightAlignment) {
                LimelightPipeline pipeline;
                if(startPos.pieceType == PieceType.CUBE) {
                    pipeline = LimelightPipeline.FIDUCIAL;
                } else {
                    pipeline = LimelightPipeline.REFLECTORS;
                }
                commands.add(new CenterLimelightCrosshairsCommand(drive, pos.limelight, pipeline));
            }

            // 2.b
            ScoreLevel actualScoreLevel;
            if(heldGamePiece == startPos.pieceType) {
                actualScoreLevel = selectedScoreLevel;
            } else {
                actualScoreLevel = ScoreLevel.LEVEL_HYBRID;
            }

            // TODO: 2.3: Score the preload in the determined level
            //       Cannot implement until the ArmSubsystem has been implemented
        }

        // 3
        String driveToMobPointPathName = String.format("%s to %s", startPos.name(), startPos.mobPos.name());
        var driveToMobPointCommand = PathFollowingUtils.getFollowTrajectoryCommand(drive, pos, driveToMobPointPathName, true);
        commands.add(driveToMobPointCommand);

        // 4
        if(chargeStationDocking) {
            String driveToChrgPathName = String.format("%s to CHRG", startPos.mobPos.name());
            var driveToChrgPointCommand = PathFollowingUtils.getFollowTrajectoryCommand(drive, pos, driveToChrgPathName, false);
            commands.add(driveToChrgPointCommand);

            // 4.a
            if(chargeStationEngagement) {
                commands.add(new BalanceScaleCommand(drive, navx));
            }
        }

        return new SequentialCommandGroup(commands.toArray(Command[]::new));
    }
}
