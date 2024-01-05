package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystem.ArmSubsystem;
// import frc.robot.Subsystem.TunePIDSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double position;
  private final PIDController armPidController;
  // private final TunePIDSubsystem tunePIDSubsystem;
  private final Supplier<Double> armAdjust;
  // TODO: ask: do we need this??? , Supplier<Double> armAdjust, Supplier<Double> wristAdjust
  public ArmCommand(ArmSubsystem armSubsystem, double position, Supplier<Double> armAdjust) {
    this.armSubsystem = armSubsystem;
        this.position = position;
        //this.telePidController = new PIDController(0.00, 0.00, 0.00);
        // TODO: change the value of the arm PID
        armPidController = new PIDController(0.05, 0.00, 0.00);
        // this.tunePIDSubsystem = new TunePIDSubsystem(armPidController, "arm PID");

        //this.armPidController = new PIDController(0.00, 0.00, 0.00);
        //this.wristPidController = new PIDController(0.00, 0.00, 0.00);
        this.armAdjust = armAdjust;
        addRequirements(armSubsystem);
        // addRequirements(tunePIDSubsystem);

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // tunePIDSubsystem.updatePIDValues();
        double armSet  = position + (this.armAdjust.get());//was this.armAdjust.get()*-10 //if needed, change it back
        double armOutput = armPidController.calculate(armSubsystem.getArmPositionDegree(), armSet);
        armOutput = (armOutput > .2)?.2:(armOutput< -.2)?-.2:armOutput; //chagne the .7 if needed, it's the max and min percent output in decimal form?
        this.armSubsystem.setArmMotor(-armOutput);
        SmartDashboard.putNumber("setArmMotor val", -armOutput);
        SmartDashboard.putNumber("kP", armPidController.getP());
        SmartDashboard.putNumber("kI", armPidController.getI());
        SmartDashboard.putNumber("kD", armPidController.getD());
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
