

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Subsystem.ArmSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;


public class New_ArmPIDSubsystem extends PIDSubsystem {
  /** Creates a new New_ArmPIDSubsystem. */
  private final RelativeEncoder armEncoder;
    CANSparkMax arm = new CANSparkMax(ArmConstants.arm_ID, MotorType.kBrushless);

  public New_ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        arm.restoreFactoryDefaults();
        arm.setInverted(ArmConstants.k_MOTORS_REVERSED);
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        arm.burnFlash();

        this.armEncoder = this.arm.getEncoder(Type.kHallSensor, 42);

        this.arm.burnFlash();

        SmartDashboard.putNumber("arm voltage in v", (Double)arm.getBusVoltage());
        SmartDashboard.putNumber("Arm output current in ams", arm.getOutputCurrent());
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    // dobule armSet = setpoint + ()
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
