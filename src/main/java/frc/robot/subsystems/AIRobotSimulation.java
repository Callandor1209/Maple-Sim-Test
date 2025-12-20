// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.commands.AutoCommand1;
import frc.robot.commands.DriveTrainDefaultCommand;

public class AIRobotSimulation extends SubsystemBase {
    //had to use shootersubsystem values again because of import conflicts between this and the other unit class needed above for inchesToMeters
      Distance distance = Robot.SHOOTER_SUBSYSTEM.distance;
  LinearVelocity linearVelocity;
    /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };

    public  static SelfControlledSwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;
    private final CommandPS5Controller controller2 = new CommandPS5Controller(1);
    public  static VisionSubsystem visionSubsystem;
    public  static IntakeSubsystem intakeSubsystem;
    public  static ShooterSubsystem shooterSubsystem;
    
        public AIRobotSimulation(int id,String name) {
            this.id = id;
            this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
            this.driveSimulation = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(
                DriveTrainSimulationConfig.Default(), 
                queeningPose
            ));
            this.visionSubsystem = new VisionSubsystem(driveSimulation.getDriveTrainSimulation(),name);
            SimulatedArena.getInstance().addDriveTrainSimulation(
                driveSimulation.getDriveTrainSimulation()
            );
            this.intakeSubsystem = new IntakeSubsystem(driveSimulation.getDriveTrainSimulation());
            this.shooterSubsystem = new ShooterSubsystem(intakeSubsystem, driveSimulation.getDriveTrainSimulation());
            //new AutoCommand1(driveSimulation.getDriveTrainSimulation(), visionSubsystem, intakeSubsystem, shooterSubsystem).schedule();
        }
    
           // PathPlanner configuration
        private static final RobotConfig PP_CONFIG = new RobotConfig(
                55.0, // Robot mass in kg
                8.0,  // Robot MOI
                new ModuleConfig(
                        Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1), // Swerve module config
                new Translation2d[]{    new Translation2d(0.286, 0.286),    
                    new Translation2d(0.286, -0.286),
                    new Translation2d(-0.286, 0.286),  
                    new Translation2d(-0.286, -0.286)  } 
        );
    
        // PathPlanner PID settings
        private final PPHolonomicDriveController driveController =
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));
    
        /** Follow path command for opponent robots */
        private Command opponentRobotFollowPath(PathPlannerPath path) {
            return new FollowPathCommand(
                    path, // Specify the path
                    // Provide actual robot pose in simulation, bypassing odometry error
                    driveSimulation::getActualPoseInSimulationWorld,
                    // Provide actual robot speed in simulation, bypassing encoder measurement error
                    driveSimulation::getActualSpeedsRobotRelative,
                    // Chassis speeds output
                    (speeds, feedforwards) -> 
                        driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                    driveController, // Specify PID controller
                    PP_CONFIG,       // Specify robot configuration
                    // Flip path based on alliance side
                    () -> DriverStation.getAlliance()
                        .orElse(DriverStation.Alliance.Blue)
                        .equals(DriverStation.Alliance.Red),
                    this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
            );
        }
    
        private Command feedShot() {
        return Commands.runOnce(() -> SimulatedArena.getInstance()
                .addGamePieceProjectile(new NoteOnFly(        // Specify the position of the chassis when the note is launched
                Robot.DRIVETRAIN_SUBSYSTEM.getPose().getTranslation(),
                // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                new Translation2d(0.2, 0),
                // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
                Robot.DRIVETRAIN_SUBSYSTEM.getMeasuredSpeeds(),
                // The shooter facing direction is the same as the robot’s facing direction
                Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation(),
                        // Add the shooter’s rotation,
                // Initial height of the flying note
                distance,
                // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
                    Robot.SHOOTER_SUBSYSTEM.getVelocity(),
                // The angle at which the note is launched
                Robot.SHOOTER_SUBSYSTEM.distance2).enableBecomeNoteOnFieldAfterTouchGround().asAmpShotNote(null)
                        .enableBecomeNoteOnFieldAfterTouchGround()));
    }
    
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };
    
    /** Joystick drive command for opponent robots */
    private Command joystickDrive(CommandPS5Controller controller)  {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
            -controller.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
            -controller.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
            -controller.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond));
    
    // Obtain driverstation facing for opponent driver station
    final Supplier<Rotation2d> opponentDriverStationFacing = () ->
            FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));
    
    return Commands.run(() -> {
            // Calculate field-centric speed from driverstation-centric speed
            final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    joystickSpeeds.get(),
                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                            .plus(Rotation2d.fromDegrees(180)));
            // Run the field-centric speed
            driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
            }, this)
            // Before the command starts, reset the robot to a position inside the field
            .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id])));
    }
    
    
    public void buildBehaviorChooser(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1,
            XboxController controller) {
                System.err.println("Starting sim");
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable =
                () -> Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(queeningPose), this)
                        .andThen(Commands.runOnce(() -> driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                        .ignoringDisable(true);
    
        // Option to disable the robot
        behaviorChooser.setDefaultOption("Disable", disable.get());
    
        behaviorChooser.addOption("Joystick Drive", joystickDrive(controller2));
    
        // Option to auto-cycle the robot
        //behaviorChooser.addOption(
             //   "Auto Cycle", new AutoCommand1(driveSimulation.getDriveTrainSimulation(), visionSubsystem, intakeSubsystem, shooterSubsystem));
    
        // Option to manually control the robot with a joystick
    
        // Schedule the command when another behavior is selected
        behaviorChooser.onChange((Command::schedule));
    
        // Schedule the selected command when teleop starts
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
    
        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(disable.get());
    
        SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
    }
    
    /** Get the command to auto-cycle the robot relatively */
    private Command getAutoCycleCommand(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1) {
        final SequentialCommandGroup cycle = new SequentialCommandGroup();
        final Pose2d startingPose = new Pose2d(
                segment0.getStartingDifferentialPose().getTranslation(),
                segment0.getIdealStartingState().rotation());
    
        cycle.addCommands(
                opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
        cycle.addCommands(
                opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));
    
        return cycle.repeatedly()
                .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
    }
    
    public static final AIRobotSimulation[] instances = new AIRobotSimulation[3]; // you can create as many opponent robots as you needs
    public static void startOpponentRobotSimulations() {
        try {
            instances[0] = new AIRobotSimulation(0,"0");
            instances[0].buildBehaviorChooser(
                    PathPlannerPath.fromPathFile("Path1"),
                    new DriveTrainDefaultCommand(null),
                PathPlannerPath.fromPathFile("Path2"),
                Commands.none(),
                new XboxController(2));

        instances[1] = new AIRobotSimulation(1,"1");
        instances[1].buildBehaviorChooser(
                PathPlannerPath.fromPathFile("Path1"),
                new DriveTrainDefaultCommand(null),
                PathPlannerPath.fromPathFile("Path2"),
                Commands.none(),
                new XboxController(3));

        
        instances[2] = new AIRobotSimulation(2,"2");
        instances[2].buildBehaviorChooser(
                        PathPlannerPath.fromPathFile("Path1"),
                        new DriveTrainDefaultCommand(null),
                        PathPlannerPath.fromPathFile("Path2"),
                        Commands.none(),
                        new XboxController(4));
    } catch (Exception e) {
        DriverStation.reportError("Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
    }
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    visionSubsystem.simUpdate();
  }
}
