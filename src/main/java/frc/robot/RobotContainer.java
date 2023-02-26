// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultVisionCommand;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.auto.BalanceScaleCommand;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystems.*;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.PathFollowingUtils;

public class RobotContainer {
    // Sensors
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    private MoInput input = new SingleControllerInput(Constants.F310);

    // Subsystems
    private VisionSubsystem visionSubsystem = new VisionSubsystem();
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arms = new ArmSubsystem();

    // Commands
    private BalanceScaleCommand balanceScaleCommand = new BalanceScaleCommand(drive, gyro);

    private Command pathFollowLinearX = PathFollowingUtils.getFollowTrajectoryCommand(drive, positioning, "Linear X", true);
    private Command pathFollowCurved = PathFollowingUtils.getFollowTrajectoryCommand(drive, positioning, "Curve", true);
    private Command pathFollowFigEight = PathFollowingUtils.getFollowTrajectoryCommand(drive, positioning, "FigureEight", true);

    private DefaultVisionCommand defaultVisionCommand = new DefaultVisionCommand(visionSubsystem);
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, input);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        drive.setDefaultCommand(driveCommand);
        visionSubsystem.setDefaultCommand(defaultVisionCommand);

        autoChooser.setDefaultOption("Balance Scale", balanceScaleCommand);
        autoChooser.addOption("Path: Linear X", pathFollowLinearX);
        autoChooser.addOption("Path: Curve", pathFollowCurved);
        autoChooser.addOption("Path: Figure 8", pathFollowFigEight);

        MoShuffleboard.getInstance().matchTab.add("Auto Chooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
