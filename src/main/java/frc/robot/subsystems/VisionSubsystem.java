// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  public VisionSystemSim visionSim;
  SimCameraProperties cameraProperties = new SimCameraProperties();
  PhotonCamera frontCamera;
  PhotonCameraSim cameraSim;
  double targetYaw;
  SwerveDriveSimulation DRIVETRAIN;
  public List<VisionTargetSim> targetSimlistNotes = new ArrayList<>();


  int i;
  int x = 0;

  double yaw = 0;
  double area;
  int tagId;

  TargetModel targetModel = new TargetModel(0.5, 0.25);
        Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d());

  public VisionSubsystem(SwerveDriveSimulation DRIVETRAIN, String name) {
    if(Robot.isSimulation()){
      this.DRIVETRAIN = DRIVETRAIN;
     visionSim = new VisionSystemSim("CameraSimSystem" + name );
     frontCamera = new PhotonCamera("Camera");
     cameraSim = new PhotonCameraSim(frontCamera, cameraProperties);
  
     visionSim.addCamera(cameraSim, new Transform3d(0,0,0, new Rotation3d(0,0,Math.toRadians(270))));
cameraSim.enableRawStream(true);

cameraSim.enableDrawWireframe(true);


  }
}

  public void simUpdate() {
    visionSim.update(DRIVETRAIN.getSimulatedDriveTrainPose());

  }

  @Override
  public void periodic() {
    if(!DriverStation.isEnabled()){
      return;
    }
    // This method will be called once per scheduler run

    //once, when the robot first activates the vision targets are added to the sim

      if(x==5){
        updateTargets();
        x= 1;
      }
      x++;
      


  
    List<PhotonPipelineResult>  pipelineResult = frontCamera.getAllUnreadResults();
    if(!pipelineResult.isEmpty()){
    PhotonPipelineResult singularResult = pipelineResult.get(pipelineResult.size() - 1);
    area = 0;
    if(singularResult.hasTargets()){
    PhotonTrackedTarget bestTarget = singularResult.getBestTarget();
     targetYaw = bestTarget.getYaw();
      area = bestTarget.getArea();
      tagId = bestTarget.getFiducialId();
    }

  }
  else{
    targetYaw = 0;
    area = 0;
  }
  


  }

  public int returnTagId(){
    return tagId;
  }

  public double returnYaw(){
    return targetYaw;
  }

  public void updateTargets(){
       yaw = DRIVETRAIN.getSimulatedDriveTrainPose().getRotation().getRadians() + Math.toRadians(90);

    for (int i= 0; SimulatedArena.getInstance().gamePiecesOnField().size() -1 >= i && DriverStation.isEnabled() && x > 0 && Robot.isSimulation(); i++) {
      
      Pose3d targetPose = SimulatedArena.getInstance().getGamePiecesPosesByType("Note").get(i);  
      targetPose = new Pose3d(targetPose.getTranslation(), new Rotation3d(0,0,yaw));
      
      if(i >= targetSimlistNotes.size()){
          targetSimlistNotes.add(i, new VisionTargetSim(targetPose, targetModel));
          visionSim.addVisionTargets(targetSimlistNotes.get(i));
        }


      targetSimlistNotes.get(i).setPose(targetPose);
      
    }

  }



  public double returnArea(){
    return area;
  }

 
}

