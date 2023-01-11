// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

public class RobotContainer {

  private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem drive = new DriveSubsystem(gyro);

  public RobotContainer() {
    configureBindings();

    try {
      visionSubsystem = new VisionSubsystem();
    } catch(VideoException e) {
      // If the visionSubsystem fails to initialize (for example, if the webcam is accidentally unplugged),
      // we shouldn't crash the whole robot. We should still be able to at least drive.
      System.out.println("Exception when initializing the VisionSubsystem:");
      e.printStackTrace();
    }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
