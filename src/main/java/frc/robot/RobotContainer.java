// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystems.*;

public class RobotContainer {

  private AHRS gyro;

  private VisionSubsystem visionSubsystem = new VisionSubsystem();
  private DriveSubsystem drive = new DriveSubsystem();

  private MoInput input = new SingleControllerInput(Constants.F310);

  private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, input);

  public RobotContainer() {
    configureBindings();

    if(RobotBase.isReal()) {
      // The navx and USBCamera classes don't play nice with the simulator,
      // so we can only create instances if we're running on the real robot.

      gyro = new AHRS(SerialPort.Port.kMXP);
      drive.initOdometry(gyro);

      visionSubsystem.init();
    }

    drive.setDefaultCommand(driveCommand);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
