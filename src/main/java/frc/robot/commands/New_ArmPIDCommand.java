// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.New_ArmPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class New_ArmPIDCommand extends PIDCommand {

  public New_ArmPIDCommand(New_ArmPIDSubsystem armSubsystem, double position, DoubleSupplier armAdjust) 
  {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        armAdjust,
        // This should return the setpoint (can also be a constant)
        position,
        // This uses the output
        output -> {
          // Use the output here
          double armOutput = output;
          armOutput = (armOutput > .2)?.2:(armOutput< -.2)?-.2:armOutput; //chagne the .7 if needed, it's the max and min percent output in decimal form?
        //   armSubsystem.setArmMotor(-armOutput);
        },
        armSubsystem
        );
        addRequirements(armSubsystem);
    }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
