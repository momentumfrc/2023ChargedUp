// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TeleopIntakeCommand;
import frc.robot.commands.auto.BalanceScaleCommand;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoBuilder;

public class RobotContainer {
    // Sensors
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    private MoInput input = new SingleControllerInput(Constants.F310);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arms = new ArmSubsystem(input);
    private IntakeSubsystem intake = new IntakeSubsystem();

    // Commands
    private TeleopArmCommand armCommand = new TeleopArmCommand(arms, input);
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, input);
    private TeleopIntakeCommand intakeCommand = new TeleopIntakeCommand(input, intake);

    private AutoBuilder autoBuilder = new AutoBuilder();

    public RobotContainer() {
        configureBindings();

        drive.setDefaultCommand(driveCommand);
        intake.setDefaultCommand(intakeCommand);
        arms.setDefaultCommand(armCommand);

        autoBuilder.initShuffleboard();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return autoBuilder.buildAutoCommand(drive, positioning, gyro);
    }
}
