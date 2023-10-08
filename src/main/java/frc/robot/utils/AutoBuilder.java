package frc.robot.utils;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.auto.BalanceScaleCommand;
import frc.robot.commands.auto.CenterLimelightCrosshairsCommand;
import frc.robot.commands.auto.DriveHoldPositionCommand;
import frc.robot.commands.auto.HoldArmSetpointCommand;
import frc.robot.commands.auto.MoveArmToSetpointCommand;
import frc.robot.commands.auto.RunIntakeCommand;
import frc.robot.commands.auto.StopIntakeCommand;
import frc.robot.sensors.Limelight;
import frc.robot.sensors.Limelight.LimelightPipeline;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.utils.ArmSetpointManager.ArmSetpoint;

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
    private GenericEntry mobilityDrive;

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

        mobilityDrive = tab.add("Drive for Mobility", false)
            .withPosition(4, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    }

    private ArmSetpoint getArmSetpoint(PieceType pieceType, ScoreLevel scoreLevel) {
        switch(pieceType) {
            case NONE:
                return ArmSetpoint.STOW;
            case CONE:
                switch(scoreLevel) {
                    case LEVEL_NONE:
                        return ArmSetpoint.STOW;
                    case LEVEL_HYBRID:
                        return ArmSetpoint.CONE_LOW;
                    case LEVEL_LOW:
                        return ArmSetpoint.CONE_MED;
                    case LEVEL_HIGH:
                        return ArmSetpoint.CONE_HIGH;
                }
            case CUBE:
                switch(scoreLevel) {
                    case LEVEL_NONE:
                        return ArmSetpoint.STOW;
                    case LEVEL_HYBRID:
                        return ArmSetpoint.CUBE_LOW;
                    case LEVEL_LOW:
                        return ArmSetpoint.CUBE_MED;
                    case LEVEL_HIGH:
                        return ArmSetpoint.CUBE_HIGH;
                }
        }
        return ArmSetpoint.STOW;
    }


    private RunIntakeCommand.IntakeDirection getIntakeDirection(PieceType pieceType) {
        switch(pieceType) {
            case CUBE:
                return RunIntakeCommand.IntakeDirection.CUBE_EXHAUST;
            case CONE:
                return RunIntakeCommand.IntakeDirection.CONE_EXHAUST;
            case NONE:
            default:
                return RunIntakeCommand.IntakeDirection.CUBE_EXHAUST;
        }
    }

    public Command buildAutoCommand(DriveSubsystem drive, PositioningSubsystem pos, ArmSubsystem arm, IntakeSubsystem intake, AHRS navx) {
        boolean masterAutoSwitch = this.masterAutoSwitch.getBoolean(false);
        StartingPosition startPos = this.startPosChooser.getSelected();
        ScoreLevel selectedScoreLevel = this.scoreLevelChooser.getSelected();
        PieceType heldGamePiece = this.heldPieceType.getSelected();
        boolean limelightAlignment = this.limelightAlignment.getBoolean(false);
        boolean chargeStationDocking = this.chargeStationDocking.getBoolean(false);
        boolean chargeStationEngagement = this.chargeStationEngagement.getBoolean(false);

        // 1: If the master autonomous switch is disabled, exit this algorithm
        if(!masterAutoSwitch) {
            return new RunCommand(() -> {});
        }

        /*
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new MoveArmToSetpointCommand(arm, ArmSetpoint.CUBE_MED),
                        new StopIntakeCommand(intake)
                    ),
                    new ParallelDeadlineGroup(
                        new RunIntakeCommand(intake, RunIntakeCommand.IntakeDirection.CUBE_EXHAUST),
                        new HoldArmSetpointCommand(arm, ArmSetpoint.CUBE_MED)
                    ),
                    new ParallelDeadlineGroup(
                        new MoveArmToSetpointCommand(arm, ArmSetpoint.STOW),
                        new StopIntakeCommand(intake)
                    )
                ),
                new DriveHoldPositionCommand(drive)
            ); */

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        // 2: If the starting position is a GRID and the specified scoring level is not LEVEL_NONE
        if(startPos.pieceType != PieceType.NONE && selectedScoreLevel != ScoreLevel.LEVEL_NONE) {
            // 2.a: If limelight alignment is enabled, perform fine-alignment based on limelight
            //      crosshairs
            if(limelightAlignment) {
                LimelightPipeline pipeline;
                if(startPos.pieceType == PieceType.CUBE) {
                    pipeline = LimelightPipeline.FIDUCIAL;
                } else {
                    pipeline = LimelightPipeline.REFLECTORS;
                }
                autoCommand.addCommands(new ParallelDeadlineGroup(
                    new CenterLimelightCrosshairsCommand(drive, pos.limelight, pipeline),
                    new HoldArmSetpointCommand(arm, ArmSetpoint.STOW),
                    new StopIntakeCommand(intake)
                ));
            }

            // 2.b: Determine the actual scoring level
            ScoreLevel actualScoreLevel;
            // 2.b.i: If the game piece type and starting position agree, then use the specified
            //        scoring level
            if(heldGamePiece == startPos.pieceType) {
                actualScoreLevel = selectedScoreLevel;
            }
            // 2.b.ii: Otherwise, if the selected starting position and game piece type are
            //         incompatible, then use the HYBRID NODE
            else {
                actualScoreLevel = ScoreLevel.LEVEL_HYBRID;
            }

            // 2.c: Score the preloaded game piece in the determined level
            ArmSetpoint scoreSetpoint = getArmSetpoint(heldGamePiece, actualScoreLevel);
            RunIntakeCommand.IntakeDirection intakeDirection = getIntakeDirection(heldGamePiece);

            autoCommand.addCommands(new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new MoveArmToSetpointCommand(arm, scoreSetpoint),
                        new StopIntakeCommand(intake)
                    ),
                    new ParallelDeadlineGroup(
                        new RunIntakeCommand(intake, intakeDirection),
                        new HoldArmSetpointCommand(arm, scoreSetpoint)
                    ),
                    new ParallelDeadlineGroup(
                        new MoveArmToSetpointCommand(arm, ArmSetpoint.STOW),
                        new StopIntakeCommand(intake)
                    )
                ),
                new DriveHoldPositionCommand(drive)
            ));
        }

        SequentialCommandGroup driveCommands = new SequentialCommandGroup();

        if(mobilityDrive.getBoolean(false)) {

            // 3: Drive to the mobility position that corresponds to the specified starting position
            String driveToMobPointPathName = String.format("%s to %s", startPos.name(), startPos.mobPos.name());
            var driveToMobPointCommand = PathFollowingUtils.getFollowTrajectoryCommand(drive, pos, driveToMobPointPathName, true);
            driveCommands.addCommands(driveToMobPointCommand);

            // 4: If charge station docking has been enabled, then drive to the charge station position
            if(chargeStationDocking) {
                String driveToChrgPathName = String.format("%s to CHRG", startPos.mobPos.name());
                var driveToChrgPointCommand = PathFollowingUtils.getFollowTrajectoryCommand(drive, pos, driveToChrgPathName, false);
                driveCommands.addCommands(driveToChrgPointCommand);

                // 4.a: If charge station engagement is enabled, then try to  balance the charge station
                if(chargeStationEngagement) {
                    driveCommands.addCommands(new BalanceScaleCommand(drive, navx));
                }
            }
        }

        driveCommands.addCommands(new DriveHoldPositionCommand(drive));

        autoCommand.addCommands(new ParallelCommandGroup(
            driveCommands,
            new HoldArmSetpointCommand(arm, ArmSetpoint.STOW),
            new StopIntakeCommand(intake)
        ));

        return autoCommand;

    }
}
