package frc.robot.autos;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;

public class DrivePathAuto extends Command {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private CommandSwerveDrivetrain drivetrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private HolonomicDriveController controller;
    private Trajectory trajectory;
    Pose2d startPose;
    Pose2d endPose;
    private Pose2d currentPose;

    private Timer timer = new Timer();
    private double prevTime = 0.0;
    private double driveTime = DriveConstants.AutoModeDriveTime;//3.25;

    /**
     * This auto will have the robot drive forwards
     * 
     * There are many ways to write autos, this form will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use a single timer gate, after the robot has finished driving for the first 3.25 
     * seconds it will stop moving. You may wish for the robot to move more or less depending on
     * your use case.
     * 
     * @param drive
     */
    public DrivePathAuto(CommandSwerveDrivetrain drivetrain)
    {
        this.drivetrain = drivetrain;

        controller = new HolonomicDriveController(
            new PIDController(DriveConstants.Hoolo_X_kP, DriveConstants.Hoolo_X_kI, DriveConstants.Hoolo_X_kD),
            new PIDController(DriveConstants.Hoolo_Y_kP, DriveConstants.Hoolo_Y_kI, DriveConstants.Hoolo_Y_kD),
            new ProfiledPIDController(DriveConstants.Hoolo_Rot_kP, DriveConstants.Hoolo_Rot_kI, DriveConstants.Hoolo_Rot_kD,
                new TrapezoidProfile.Constraints(6.28, 3.14)));
        controller.setTolerance(DriveConstants.Hoolo_Tolerance);

        addRequirements(drivetrain);
    }

    public void generateTrajectory() {
        startPose = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
            Rotation2d.fromDegrees(-180));
        endPose = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
            Rotation2d.fromDegrees(-160));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
//        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setKinematics(drivetrain.getKinematics());
        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
//        config.setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);
    }

    @Override
  public void initialize() {
    System.out.println("Auto-initialize");
    prevTime = 0.0;
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second), here we will just drive forwards
  @Override
  public void execute() {
    // drive forward at 30% speed
    double time = timer.get();
    if(time < driveTime)
    {
        double delta = time - prevTime;

        double speed = drivetrain.allianceColor == Alliance.Red ? DriveConstants.AutoModeSpeed : -DriveConstants.AutoModeSpeed;

        // Sample the trajectory at 3.4 seconds from the beginning.
        Trajectory.State goal = trajectory.sample(time);
        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        //double angle = targetRot.getAngle();
        //Rotation2d rot = Rotation2d.fromDegrees(70.0);
        Rotation2d rot = goal.poseMeters.getRotation();

        ChassisSpeeds adjustedSpeeds = controller.calculate(currentPose, goal, rot);
        drivetrain.setChassisSpeeds(adjustedSpeeds);

    //   drivetrain.setControl(drive.withVelocityX(speed)
    //     .withVelocityY(0)
    //     .withRotationalRate(0));
    }
    prevTime = time;
}

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    drivetrain.setControl(drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0));

    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds driving time in seconds, when it has this will return true indicating
    // this command is finished
    return controller.atReference() || timer.get() >= driveTime;
  }
}
