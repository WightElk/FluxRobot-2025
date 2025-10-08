// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.generated.TunerConstants.ConstantCreator;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  public final SwerveDrivetrainConstants DrivetrainConstants;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final CommandSwerveDrivetrain drivetrain;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  protected final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Sensitivity sensitivityPos = 
      new Sensitivity(OperatorConstants.Threshold, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitX);

  private final Sensitivity sensitivityRot =
      new Sensitivity(OperatorConstants.Threshold, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitRot);

  protected final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(String canBusName, int pigeonId, Pigeon2Configuration pigeonConfigs,
    SwerveModuleConfig frontLeft, SwerveModuleConfig frontRight, SwerveModuleConfig backLeft, SwerveModuleConfig backRight)
  {
    DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(canBusName)
      .withPigeon2Id(pigeonId)
      .withPigeon2Configs(pigeonConfigs);

    drivetrain = createDrivetrain(frontLeft, frontRight, backLeft, backRight);
    drivetrain.registerTelemetry(logger::telemeterize);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  protected void configureBindings()
  {
    /*  Example: How to bind commands to triggers
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    */

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(
            // Drive forward with negative Y (forward)
            MaxSpeed * sensitivityPos.transfer(-driverController.getLeftY())
          )
          .withVelocityY(
            // Drive left with negative X (left)
            MaxSpeed * sensitivityPos.transfer(-driverController.getLeftX())
          )
          .withRotationalRate(
            // Drive counterclockwise with negative X (left)
            -driverController.getRightX() * MaxAngularRate
          )
      )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }

  private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(SwerveModuleConfig constants)
  {
    return ConstantCreator.createModuleConstants(
      constants.steerMotorId, constants.driveMotorId, constants.encoderId,
      constants.encoderOffset,
      constants.xPos, constants.yPos,
      TunerConstants.kInvertLeftSide,
      constants.steerMotorInverted, constants.encoderInverted
    );
  }

  // Creates a CommandSwerveDrivetrain instance.
  // This should only be called once in your robot program,.
  private CommandSwerveDrivetrain createDrivetrain(SwerveModuleConfig frontLeft, SwerveModuleConfig frontRight, SwerveModuleConfig backLeft, SwerveModuleConfig backRight)
  {
    return new CommandSwerveDrivetrain(
        DrivetrainConstants,
        createModuleConstants(frontLeft), createModuleConstants(frontRight),
        createModuleConstants(backLeft), createModuleConstants(backRight)
    );
  }
}
