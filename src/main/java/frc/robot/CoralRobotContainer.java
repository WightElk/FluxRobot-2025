package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.TrayConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TraySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.TrayInOutCommand;
import frc.robot.commands.RawTrayCommand;
import frc.robot.autos.DriveForwardAuto;

/**
 * Robot with Coral Elevator
 */
public class CoralRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.CoralRobot.systemCANBus);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final TraySubsystem tray = new TraySubsystem(canBus);
  private final Lights lights = new Lights(RobotConfig.CoralRobot.systemCANBus);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);

  public CoralRobotContainer() {
    super(RobotConfig.CoralRobot, true);

    configureBindings();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

//    SmartDashboard.putBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);

    useTwoControllers = SmartDashboard.getBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;
    // Elevator control bindings
    controller.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
    controller.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
    controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    controller.y().onTrue(new RunCommand(() -> elevator.moveToLevel2(), elevator));
    controller.b().onTrue(new RunCommand(() -> elevator.moveToLevel1(), elevator));
    controller.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));

    // Tray control bindings
    controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new TrayInOutCommand(tray, () -> controller.getRightTriggerAxis()));
    controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RawTrayCommand(tray, () -> - TrayConstants.Speed * controller.getLeftTriggerAxis()));
    controller.leftBumper().whileTrue(new RawTrayCommand(tray, () -> TrayConstants.BackwardSpeed));
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoDriveForward;
  }
}
