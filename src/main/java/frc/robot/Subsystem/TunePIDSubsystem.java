// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystem;

// import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.networktables.NetworkTableEntry;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

// import java.util.ArrayList;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import java.util.List;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Subsystem.TunePIDSubsystem;

// public class TunePIDSubsystem extends SubsystemBase {
//   private final ShuffleboardTab pidTab = Shuffleboard.getTab("Tune PID");
//   private final List<PIDController> pidControllers = new ArrayList<>();
//   private final List<SimpleWidget> pWidgets = new ArrayList<>();
//   private final List<SimpleWidget> iWidgets = new ArrayList<>();
//   private final List<SimpleWidget> dWidgets = new ArrayList<>();

//     public TunePIDSubsystem(PIDController pid, String name)
//     {
//         addNewPIDController(pid, name);
//     }

//     public TunePIDSubsystem()
//     {
        
//     }

//   public void addNewPIDController(PIDController pid, String name) {
//     pidControllers.add(pid);
//     ShuffleboardLayout pidLayout = pidTab.getLayout(name, BuiltInLayouts.kList);
//     pWidgets.add(pidLayout.add("P Value", 0).withWidget(BuiltInWidgets.kNumberBar));
//     iWidgets.add(pidLayout.add("I Value", 0).withWidget(BuiltInWidgets.kNumberBar));
//     dWidgets.add(pidLayout.add("D Value", 0).withWidget(BuiltInWidgets.kTextView));
//   }

//   public void updatePIDValues() {
//     for (int i = 0; i < pidControllers.size(); i++) {
//       double kP = pWidgets.get(i).getEntry().getDouble(5);
//       double kI = iWidgets.get(i).getEntry().getDouble(0);
//       double kD = dWidgets.get(i).getEntry().getDouble(0);
//       pidControllers.get(i).setPID(kP, kI, kD);
//     }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
