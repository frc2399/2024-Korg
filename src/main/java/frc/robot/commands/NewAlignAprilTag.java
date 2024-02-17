// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NewAlignAprilTag extends Command {
    DriveSubsystem driveSubsystem;
    Vision vision;
    private boolean seesSpeaker = false;
    Rotation2d yawDiff;
    private int desiredTagID;
  
  public NewAlignAprilTag(DriveSubsystem m_driveSubsystem, Vision m_visionSubsystem, int speakerID) {
    //makes the global variables the same as the ones intook from the call
    driveSubsystem = m_driveSubsystem;
    vision = m_visionSubsystem;
    desiredTagID = speakerID;
    addRequirements(m_driveSubsystem, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //looks to see if it can see the speaker. if it can't, exits.
    for (PhotonTrackedTarget result : vision.getCameraResult().getTargets()) {
        if (result.getFiducialId() == desiredTagID) {
            seesSpeaker = true;
            break; //saves a tiny bit of processing power possibly
        }
    }
    if (!seesSpeaker) {
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //gets the yawDiff from PhotonUtils
    if (seesSpeaker) {
        yawDiff = PhotonUtils.getYawToPose(
            vision.robotPose, VisionConstants.kFieldLayout.getTagPose(7).get().toPose2d());
    //turns based on if yawDiff is pos or neg 
    // TODO: test these values
        if (yawDiff.getDegrees() > 0) {
            driveSubsystem.drive(0,0,-0.1, true, true);
        } else {
            driveSubsystem.drive(0,0,0.1,true,true);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Entered end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if the yawDiff is within 5 degrees, ends the program
    if (Math.abs(yawDiff.getDegrees())<5) {
        return true;
    } else {
        return false;
    }
  }
}