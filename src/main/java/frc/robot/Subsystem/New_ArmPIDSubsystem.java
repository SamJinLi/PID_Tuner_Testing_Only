

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import java.util.function.Supplier;


public class New_ArmPIDSubsystem extends PIDSubsystem {
  /** Creates a new New_ArmPIDSubsystem. */
  private final RelativeEncoder armEncoder;
  CANSparkMax arm = new CANSparkMax(ArmConstants.arm_ID, MotorType.kBrushless);
  // private double finalOutput = 0.0;

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
    int valToChange = 0;//TODO: fix the valTOchange, was 0
    double yOut = (output > .2)?.2:(output< -.2)?-.2:output; //chagne the .7 if needed, it's the max and min percent output in decimal form?
    double finalOut = (Math.abs(getMeasurement()) >= ArmConstants.k_SOFT_LIMIT && !((getMeasurement() < valToChange) ^ (yOut < valToChange))) ? 0 : yOut;
    // finalOut = -finalOut; // uncomment if the motor is rotating opposite direction. This flips the order.
    SmartDashboard.putNumber("Arm output raw : ",finalOut);
    arm.set(finalOut);
  }

  @Override
  public void periodic(){
    super.periodic();
    SmartDashboard.putNumber("arm voltage in v", (Double)arm.getBusVoltage());
    SmartDashboard.putNumber("Arm output current in ams", arm.getOutputCurrent());
    SmartDashboard.putNumber("Raw encoder value Spark max arm", getMeasurement());
  }

  public void setSetpoint(double setpoint, Supplier<Double> armAdjust)
  {
    double armSet = setpoint+ armAdjust.get();
    super.setSetpoint(armSet);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double angle = (armEncoder.getPosition() / 8.75); // NOTE: add whatever value after /8.75 if the value is off.
    return angle + 0; // change 0 to any off-sets
  }
}
