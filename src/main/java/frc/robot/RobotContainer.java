// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
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
  private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  private VisionSubsystem visionSubsystem = new VisionSubsystem();
  private DriveSubsystem drive = new DriveSubsystem(gyro);

  private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);

  private DefaultVisionCommand defaultVisionCommand = new DefaultVisionCommand(visionSubsystem);

  private MoInput input = new SingleControllerInput(Constants.F310);

  private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, input);

  public RobotContainer() {
    configureBindings();

    drive.setDefaultCommand(driveCommand);
    visionSubsystem.setDefaultCommand(defaultVisionCommand);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    drive.resetMaintainHeading();
  }
}
