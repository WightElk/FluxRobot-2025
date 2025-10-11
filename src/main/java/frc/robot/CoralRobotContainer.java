package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TraySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.TrayInOutCommand;

/**
 * Robot with Coral Elevator
 */
public class CoralRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.CoralRobot.systemCANBus);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final TraySubsystem tray = new TraySubsystem(canBus);
  private final VisionSubsystem vision;

  // Set to 1 to use driver controller for everything, 2 to use operator controller for elevator and tray
  private boolean useTwoControllers = OperatorConstants.UseTwoControllers;
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  public CoralRobotContainer() {
    super(RobotConfig.CoralRobot);

    configureBindings();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

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
    controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new TrayInOutCommand(tray, () -> driverController.getRightTriggerAxis()));

    // Vision control bindings (driver controller only)
    // Left bumper: Drive to AprilTag (vision-guided alignment)
    driverController.leftBumper().whileTrue(new DriveToTag(vision, drivetrain));
  }

  /**
   * Gets the vision subsystem (Coral robot only - single camera).
   * @return The VisionSubsystem instance
   */
  public VisionSubsystem getVision() {
    return vision;
  }
}
