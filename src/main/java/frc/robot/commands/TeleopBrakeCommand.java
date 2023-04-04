// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.MoInput;
import frc.robot.subsystems.BrakeSubsystem;

public class TeleopBrakeCommand extends CommandBase {
    private final Supplier<MoInput> inputSupplier;
    private final BrakeSubsystem brakes;

    public TeleopBrakeCommand(BrakeSubsystem brakes, Supplier<MoInput> inputSupplier) {
        this.inputSupplier = inputSupplier;
        this.brakes = brakes;

        this.addRequirements(brakes);
    }

    @Override
    public void initialize() {
        this.brakes.setBrakesExtended(false);
    }

    @Override
    public void execute() {
        var input = inputSupplier.get();
        if (input.getShouldBrake()) {
            this.brakes.setBrakesExtended(true);
        }
    }
}
