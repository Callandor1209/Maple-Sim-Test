package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class MapleSimSwerve implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    public MapleSimSwerve(boolean configureAutoBuilder) {
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default();

        // Creating the SelfControlledSwerveDriveSimulation instance
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(3, 3, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);
        if(configureAutoBuilder){
          configureAutoBuilder();
        }
    }

 


    private void configureAutoBuilder()  {
            try {
                AutoBuilder.configure(
                        // Use APIs from SwerveDrive interface
                        this::getPose, 
                        this::setPose,
                        this::getMeasuredSpeeds,
                        (speeds) -> this.drive(speeds, false, true),
   
                        // Configure the Auto PIDs
                        new PPHolonomicDriveController(
                                             // PID constants for translation
                        new PIDConstants(10, 0, 0),
                        // PID constants for rotation
                        new PIDConstants(7, 0, 0)),
   
                        // Specify the PathPlanner Robot Config
                        RobotConfig.fromGUISettings(),
   
                        // Path Flipping: Determines if the path should be flipped based on the robot's alliance color
                        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red),
   
                        // Specify the drive subsystem as a requirement of the command
                        this);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (org.json.simple.parser.ParseException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    



    

        @Override
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond),
                new Translation2d(),
                fieldRelative,
                false);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
        simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());

    }

    public SwerveDriveSimulation returnSwerveThing(){
        return simulatedDrive.getDriveTrainSimulation();
    }

}