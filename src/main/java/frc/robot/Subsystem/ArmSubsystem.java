// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

// Libraries import from officila code start
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import java.util.function.Supplier;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.math.controller.PIDController;
// Libraries import from officila code end

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    /** Creates a new ArmSubsystem. */
    // private final SparkMaxPIDController armPidController;
    private final RelativeEncoder armEncoder;
    CANSparkMax arm = new CANSparkMax(ArmConstants.arm_ID, MotorType.kBrushless);

    public ArmSubsystem() {

        arm.restoreFactoryDefaults();
        arm.setInverted(ArmConstants.k_MOTORS_REVERSED);
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        arm.burnFlash();

        // this.armPidController = arm.getPIDController();
        // this.armPidController.setP(importarmPidController.getP());
        // this.armPidController.setI(importarmPidController.getI());
        // this.armPidController.setD(importarmPidController.getD());
        // this.armPidController.setFF(0);
        // this.armPidController.setIZone(0);

        this.armEncoder = this.arm.getEncoder(Type.kHallSensor, 42);

        this.arm.burnFlash();

        SmartDashboard.putNumber("arm voltage in v", (Double)arm.getBusVoltage());
        SmartDashboard.putNumber("Arm output current in ams", arm.getOutputCurrent());
    }

    public void setArmMotor(double input) {
         //stop it from going too far
        
        //reduce input because adding the holding value could make it over 1
        input *= .9;
        //get holding output flips itself so we just add this
        //input += getArmHoldingOutput();
        int valToChange = 0;//TODO: fix the valTOchange, was 0

        if(Math.abs(getArmPositionDegree()) >= ArmConstants.k_SOFT_LIMIT && !((getArmPositionDegree() < valToChange ^ (input < valToChange)))){
            input = 0;
        }
        //we only set the leader cuz the follower will just do the same
        this.arm.set(input);
        //updating the shuffle board output
        SmartDashboard.putNumber("Arm output raw : ",input);
        // SmartDashboard.putNumber("arm voltage in v", (Double)arm.getBusVoltage());
        // SmartDashboard.putNumber("Arm output current in ams", arm.getOutputCurrent());

        // kP = 0.05;
        // kI = 0.2; // was 0.2
        // iLimit = 8;
        // // kI = 0;
        // kD = 0.009;
        // double error = position - armEncoder.getPosition();//-22
        // double dt = Timer.getFPGATimestamp() - lastTimestamp;
        // errorRate = (error - lastError) / dt;

        // SmartDashboard.putNumber("Raw encoder value Spark max arm", getArmPositionDegree());
        // SmartDashboard.putNumber("error", error);
        // SmartDashboard.putNumber("errorrate", errorRate);
        
        // if (Math.abs(error) < iLimit) {
        //     errorSum += error * dt;
        //   }
        // double outputSpeed = kP * error + kI*errorSum + kD * errorRate;
        // if (position == zeron){
        //     outputSpeed*=0.25;
        // }
        // arm.set(outputSpeed);

        
        // lastTimestamp = Timer.getFPGATimestamp();
        // lastError = error;
        // // SmartDashboard.putNumber("arm output", input);
        // // SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
        // // SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
    }

    public double getArmPositionDegree(){
        double angle = (armEncoder.getPosition() / 8.75); // TODO: add whatever value after /8.75 if the value is off. add the beginnig value
        return angle+0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Raw encoder value Spark max arm", getArmPositionDegree());
        // SmartDashboard.putNumber("Arm output raw : ", input);
        SmartDashboard.putNumber("arm voltage in v", (Double)arm.getBusVoltage());
        SmartDashboard.putNumber("Arm output current in ams", arm.getOutputCurrent());
        SmartDashboard.putNumber("Arm Encoder Val", getArmPositionDegree());
        // This method will be called once per scheduler run
    }
}
