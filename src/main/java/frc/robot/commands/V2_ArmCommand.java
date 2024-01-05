// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.New_ArmPIDSubsystem;

public class V2_ArmCommand extends CommandBase {
  /** Creates a new V2_ArmCommand. */
  private final New_ArmPIDSubsystem armPIDSubsystem;
  private final double position;
  private final Supplier<Double> armAdjust;
  public V2_ArmCommand(New_ArmPIDSubsystem armPIDSubsystem, double position, Supplier<Double> armAdjust) {
    this.armPIDSubsystem = armPIDSubsystem;
    this.position = position;
    this.armAdjust = armAdjust;
    addRequirements(armPIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armPIDSubsystem.setSetpoint(position,armAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
