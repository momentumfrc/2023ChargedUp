// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AprilTagsVisionCommand;
import frc.robot.commands.DefaultVisionCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystems.*;
import frc.robot.utils.MoShuffleboard;
import frc.robot.utils.ShuffleboardToggle;

public class RobotContainer {

  private VisionSubsystem visionSubsystem = new VisionSubsystem();
  private DriveSubsystem drive = new DriveSubsystem();

  private DefaultVisionCommand defaultVisionCommand = new DefaultVisionCommand(visionSubsystem);
  private AprilTagsVisionCommand aprilTagsVisionCommand = new AprilTagsVisionCommand(visionSubsystem, drive);

  private MoInput input = new SingleControllerInput(Constants.F310);

  private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, input);

  private Trigger aprilTagsVisionTrigger = new ShuffleboardToggle(
    MoShuffleboard.getInstance().settingsTab,
    "Detect AprilTags",
    true
  ).getTrigger();

  public RobotContainer() {
    configureBindings();

    drive.setDefaultCommand(driveCommand);
    visionSubsystem.setDefaultCommand(defaultVisionCommand);

    aprilTagsVisionCommand.schedule();
    aprilTagsVisionTrigger.whileTrue(aprilTagsVisionCommand);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    drive.resetMaintainHeading();
  }
}
