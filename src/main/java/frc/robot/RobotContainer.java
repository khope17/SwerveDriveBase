// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTrainCommands.DefaultDriveCommand;
import frc.robot.commands.DriveTrainCommands.ResetGyro;
import frc.robot.commands.DriveTrainCommands.AutoBalance;
import frc.robot.commands.DriveTrainCommands.sturdyBaseCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  SwerveModule m_frontLeftModule;
  SwerveModule m_frontRightModule;
  SwerveModule m_backLeftModule;
  SwerveModule m_backRightModule;

  // Set to use Path Planner
  private boolean usePathPlanner = true;
  // private boolean debugSwerve = false;
  SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  SwerveAutoBuilder autoBuilder;
  List<PathPlannerTrajectory> selectedAuto;
  SendableChooser<String> AllianceChooser = new SendableChooser<>();

  // Declare our other subsystems

  // Declare our two PS4 Controllers
  // DriverStick on USB Port 0
  // OperStick on USB Port 1
  private final CommandPS4Controller driverStick = new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller operStick = new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

  // Init the HashMap for Path Planner Commands
  HashMap<String, Command> pathPlannerEventMap = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    driveTrain.setDefaultCommand(new DefaultDriveCommand(
        driveTrain,
        () -> -modifyAxis(driverStick.getLeftY() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getLeftX() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverStick.getRightX() * Constants.DRIVETRAIN_THROTTLE)
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Send our Auto Chooser options to the Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
    // SmartDashboard.putString("selectedAuto", selectedAuto);

    // Generate Path Planner Groups
    generatePathPlannerPathGroups();
    createAutoBuilder();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverStick.L3().whileTrue(new sturdyBaseCommand(driveTrain));
    driverStick.touchpad().whileTrue(new AutoBalance(driveTrain));
    driverStick.povDown().whileTrue(new RunCommand(driveTrain::zeroGyroscope, driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    usePathPlanner = SmartDashboard.getBoolean("Use PathPlanner", true);
    if (usePathPlanner) {
      return generateAutoWithPathPlanner();
    } else {
      return manuallyGenerateAuto();
    }

  }

  /**
   * Generates the Path Planner Groups and add them to the AutoChooser
   */
  private void generatePathPlannerPathGroups() {
    List<PathPlannerTrajectory> Station_2_Engage = PathPlanner.loadPathGroup("Station 2 Engage",
        new PathConstraints(2, 2));
    List<PathPlannerTrajectory> Station_1_Cube_and_Pickuptraj = PathPlanner.loadPathGroup("Station 1 Cube and Pickup",
        new PathConstraints(2, 3));
    List<PathPlannerTrajectory> Station_1_Drop_Cone_And_Pickuptraj = PathPlanner.loadPathGroup(
        "Station 1 Drop Cone and Pickup",
        new PathConstraints(2, 3));
    List<PathPlannerTrajectory> Station_3_Engage = PathPlanner.loadPathGroup("Station 3 Engage",
        new PathConstraints(2, 2));

    autoChooser.setDefaultOption("Station 2 Engage", Station_2_Engage);
    autoChooser.addOption("Station 1 Cube and Pickup", Station_1_Cube_and_Pickuptraj);
    autoChooser.addOption("Station 1 Drop Cone And Pickup", Station_1_Drop_Cone_And_Pickuptraj);
    autoChooser.addOption("Station 3 Engage", Station_3_Engage);
  }

  /**
   * Creates the AutoBuilder
   */
  private void createAutoBuilder() {

    pathPlannerEventMap = new HashMap<>();
    // pathPlannerEventMap.put("marker1", new PrintCommand("Passed marker 1"));
    // pathPlannerEventMap.put("intakeDown", new PrintCommand("Event 2"));
    pathPlannerEventMap.put("Wait", new WaitCommand(1));
    pathPlannerEventMap.put("Auto Balance", new AutoBalance(driveTrain));
    pathPlannerEventMap.put("Reset Gyro", new ResetGyro(driveTrain));
    pathPlannerEventMap.put("Sturdy Base", new sturdyBaseCommand(driveTrain));

    autoBuilder = new SwerveAutoBuilder(
        driveTrain::getPose, // Pose2d supplier
        driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        driveTrain.geKinematics(), // SwerveDriveKinematics
        new PIDConstants(Constants.kPXYController, 0.0, 0.0), // PID constants to correct for translation error (used to
                                                              // create the X and Y PID controllers)
        new PIDConstants(Constants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to
                                                                 // create the rotation controller)
        driveTrain::drive, // Module states consumer used to output to the drive subsystem
        pathPlannerEventMap,
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        driveTrain // The drive subsystem. Used to properly set the requirements of path following
                   // commands
    );
  }

  private Command generateAutoWithPathPlanner() {
    return autoBuilder.fullAuto(autoChooser.getSelected());
  }

  /**
   * Creates a command that follows a manually generated path
   * 
   * @return
   */
  private Command manuallyGenerateAuto() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND_TRAJECTORY,
        DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(driveTrain.geKinematics());

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(0, -1)),
        // new Translation2d(2, Rotation2d.fromDegrees(-90))),
        new Pose2d(1, -1, Rotation2d.fromDegrees(90)),

        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        driveTrain::getPose,
        driveTrain.geKinematics(),
        xController,
        yController,
        thetaController,
        driveTrain::drive,
        driveTrain);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> driveTrain.stopModules()));

  }

  /**
   * Checks to make sure a setting is outside of our dedband range
   * 
   * @param value    the value to check
   * @param deadband the range of our deadband
   * @return 0 if inside the deadband, the value if outside the deadband
   *         normalized
   */
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;

  }

  // Returns the alliance color that we are on
  public DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance();
  }

  // Returns the DriveTrain Subsystem to other classes
  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return driveTrain;
  }

}
