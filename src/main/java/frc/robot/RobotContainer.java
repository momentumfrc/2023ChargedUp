// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.animations.Solid;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopArmCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TeleopIntakeCommand;
import frc.robot.input.DualControllerInput;
import frc.robot.input.MoInput;
import frc.robot.input.SingleControllerInput;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoBuilder;
import frc.robot.utils.MoShuffleboard;

public class RobotContainer {

    // Sensors
    private AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    // Subsystems
    private DriveSubsystem drive = new DriveSubsystem(gyro);
    private PositioningSubsystem positioning = new PositioningSubsystem(gyro, drive);
    private ArmSubsystem arms = new ArmSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private LEDSubsystem leds = new LEDSubsystem();

    // Commands
    private TeleopArmCommand armCommand = new TeleopArmCommand(arms, this::getInput);
    private TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, positioning, this::getInput);
    private TeleopIntakeCommand intakeCommand = new TeleopIntakeCommand(intake, this::getInput);

    private AutoBuilder autoBuilder = new AutoBuilder();
    private SendableChooser<MoInput> inputChooser = new SendableChooser<>();

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public RobotContainer() {
        configureBindings();

        inputChooser.setDefaultOption("Dual Controllers", new DualControllerInput(Constants.DRIVE_F310, Constants.ARMS_F310));
        inputChooser.addOption("Single Controller", new SingleControllerInput(Constants.DRIVE_F310));
        MoShuffleboard.getInstance().settingsTab.add("Controller Mode", inputChooser);

        drive.setDefaultCommand(driveCommand);
        intake.setDefaultCommand(intakeCommand);
        arms.setDefaultCommand(armCommand);

        autoBuilder.initShuffleboard();

        positioning.limelight.setAlignmentAnimation(
            leds.getCompositor().getOpaqueView(new Solid(Color.GREEN), 100)
        );
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return autoBuilder.buildAutoCommand(drive, positioning, arms, intake, gyro);
    }
}
