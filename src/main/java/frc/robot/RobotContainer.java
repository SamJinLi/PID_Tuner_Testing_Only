// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.ArmSubsystem;

import frc.robot.Subsystem.New_ArmPIDSubsystem;
// import frc.robot.Subsystem.TunePIDSub            system;
// import frc.robot.commands.ATWAutoCmd;
// import frc.robot.commands.ATWJoystickCmd;
import frc.robot.commands.New_ArmPIDCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
//   private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final New_ArmPIDSubsystem armSubsystem = new New_ArmPIDSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
//   private final XboxController m_controller = new XboxController(0);
  // NOTE: port 1 is arm
  private final Joystick m_js = new Joystick(1);
  // NOTE: port 2 is drivetrain?
  // private final Joystick m_js2 = new Joystick(2);
  // TODO: TUNE ALL THE VALUES
  private final double test = -0.13061;
  private final double conePickUpGround = -0.40816;
  // private final double zeron = 0;//zero
  // private final double flatn = 0;//flat
  // private final double zerop = 0;
  // private final double vert = 0;
  // private final double forty5 = 0;//notc
  // private final double pickupn = 0;
  // private final double pickupp = 0;//pickup
  // private final double midn = 0;//mid cone
  // private final double midp = 0;
  // private final double highn = 0;//high cone
  // private final double highp = 0;
  // private final double stationp = 0;
  // private final double stationn = 0;
  // private final double hovern = 0;
  // private final double hoverp = 0;
  // private final double siden = 0;
  // private final double sidep = 0;
  // private final double autocubehigh = 0;
  // private final double telecubehigh = 0;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private String m_autoselected = "New Path";
  private final double pos = -3;
  private final double pos2 = -2;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    autoChooser.setDefaultOption("New Path", "New Path");
    autoChooser.addOption("MidMobA", "MidMobA");
    autoChooser.addOption("MidMobB", "MidMobB");
    autoChooser.addOption("MidClimb", "MidClimb");
    autoChooser.addOption("MidClimbMob", "MidClimbMob");
    autoChooser.addOption("Top","Top");
    autoChooser.addOption("Bot", "Bot");
    autoChooser.addOption("BotClimb", "BotClimb");
    autoChooser.addOption("FollowBot", "FollowBot");
    SmartDashboard.putData(autoChooser);
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * .5
    // ));
    // TODO: joystick control?
//     armSubsystem.setDefaultCommand(new ATWJoystickCmd(
//       armSubsystem,
//       () -> 0d,
//       () -> -m_js.getY(),
//       () -> 0d

// ));
// TODO: write the INTAKE!
// intakeSubsystem.setDefaultCommand(new IntakeDefaultCmd(
//   intakeSubsystem,
//   m_js::getTrigger,
//   () -> m_js.getRawButton(2)
// ));
// armSubsystem.setDefaultCommand(new ArmManuelCmd(
//       armSubsystem,
//       () -> -m_js.getY()
// ));
    // Configure the button bindings
    configureButtonBindings();
    // usbCamera = new UsbCamera("cam", 1);
    // CameraServer.startAutomaticCapture();
   
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    // new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheelsCmd(m_drivetrainSubsystem));
    // Back button zeros the gyroscope
    // new Trigger(m_controller::getXButtonPressed).onTrue(new AutoBalanceCommand(
    //     m_drivetrainSubsystem, m_controller::getXButton
    // ) 
    // );
    // new Trigger(m_controller::getBackButtonPressed).onTrue(new ZeroGyroscope(m_drivetrainSubsystem));
        //new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        //new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));






        
        // Change all these 
        //zero that bih!
        // TODO: startfromhere
        new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new New_ArmPIDCommand(
            armSubsystem,
            conePickUpGround,
            () -> m_js.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new New_ArmPIDCommand(
            armSubsystem,
            conePickUpGround,
            () -> m_js.getThrottle()
        ));
        // new Trigger(() -> m_js.getRawButtonPressed(5)).onTrue(new ArmCommand(
        //     tunePIDSubsystem,
        //     armSubsystem,
        //     test,
        //     () -> m_js.getThrottle()
        
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(7)).onTrue(new ArmCommand(
        //     armSubsystem,
        //     zeron,
        //     () -> m_js.getThrottle()
        // ));
        // new Trigger(() -> m_js2.getRawButtonPressed(8)).onTrue(new ArmCommand(
        //     armSubsystem,
        //     flatn,
        //     () -> m_js.getThrottle()
        
        // ));
        // new Trigger(() -> m_js.getRawButtonPressed(7)).onTrue(new ArmCommand(
        //     armSubsystem,
        //     zerop,
        //     () -> m_js.getThrottle()
        
        // ));


        //flat like a door (or your mother)
        // new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ArmCommand(
        //     armSubsystem,
        //     flatn,
        //     () -> m_js.getThrottle()
        
        // ));
        //the thing that a notc would do
        // new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ArmCommand(
        //     armSubsystem,
        //     forty5,
        //     () -> m_js.getThrottle()
        
        // ));
        //pick that shi up
//         new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new ArmCommand(
//             armSubsystem,
//             pickupn,
//             () -> m_js.getThrottle()
//         ));
//         //mid cone (negative direction)
//         new Trigger(() -> m_js.getRawButtonPressed(4)).onTrue(
//             new ArmCommand(armSubsystem,
//              autocubehigh,
//              () -> m_js.getThrottle()
// )
//         );
        
//         //high cone (negative direction)
//         new Trigger(() -> m_js.getRawButtonPressed(5)).onTrue(new ArmCommand(
//             armSubsystem,
//             highn,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new ArmCommand(
//             armSubsystem,
//             pickupp,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(4)).onTrue(new ArmCommand(
//             armSubsystem,
//             telecubehigh,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(5)).onTrue(new ArmCommand(
//             armSubsystem,
//             highp,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(6)).onTrue(new ArmCommand(
//             armSubsystem,
//             stationn,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(6)).onTrue(new ArmCommand(
//             armSubsystem,
//             stationp,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(11)).onTrue(new ArmCommand(
//             armSubsystem,
//             hoverp,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(12)).onTrue(new ArmCommand(
//             armSubsystem,
//             sidep,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(11)).onTrue(new ArmCommand(
//             armSubsystem,
//             hovern,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(12)).onTrue(new ArmCommand(
//             armSubsystem,
//             siden,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButton(10)).onTrue(new ArmCommand(
//           armSubsystem, zeron, () -> m_js.getThrottle())
//         );
//         new Trigger(() -> m_js.getRawButtonPressed(9)).onTrue(new ArmCommand(
//             armSubsystem,
//             midn,
//             () -> m_js.getThrottle()
//         ));
//         new Trigger(() -> m_js.getRawButtonPressed(9)).onTrue(new ArmCommand(
//             armSubsystem,
//             midp,
//             () -> m_js.getThrottle()
//         ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     // An ExampleCommand will run in autonomous
//     // return new SequentialCommandGroup(
//     //   new ZeroGyroscope(m_drivetrainSubsystem),
//     //   new DriveXCommand(m_drivetrainSubsystem, pos)
//     // );
//     m_autoselected = autoChooser.getSelected();
//     // return new SequentialCommandGroup(
//       // new DisableCompCmd(intakeSubsystem),
//       // //new CloseSolenoidCmd(intakeSubsystem),//for cone
//       // new ATWAutoCmd(armSubsystem, autocubehigh, null, null),
//       // //new ATWAutoCmd(armSubsystem, highn, null, null),
//       // new ShootCubeCmd(intakeSubsystem),
//       // //new CloseSolenoidCmd(intakeSubsystem),//for cone only
//       // new ATWAutoCmd(armSubsystem, zeron, null, null),
//       // new CloseSolenoidCmd(intakeSubsystem),
//     //   new PathPlannerCmd(m_drivetrainSubsystem, "ThreeMeter")
//       // new ChargingStationAuto(m_drivetrainSubsystem),
//       // new LockWheelsCmd(m_drivetrainSubsystem)
//     // );

    
//   }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

//   private static double modifyAxis(double value) {
//     // Deadband
//     value = deadband(value, 0.06);

//     // Square the axis
//     value = Math.copySign(value * value, value);

//     return value;
//   }
}
