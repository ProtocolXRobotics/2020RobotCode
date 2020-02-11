/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private double xOffset; // Positive values mean that target is to the right of the camera; negative values mean target is to the left. Measured in degrees
  private double yOffset; // Positive values mean that target is above the camera; negative values mean target is below. Measured in degrees
  private double targetArea; // Returns a value of the percentage of the image the target takes
  private double targetValue; // Sends 1 if a target is detected, 0 if none are present

  private int pipeline; // Used to identify which pipline the limelight uses (0-9)

  // Create a network table for the limelight
  private final NetworkTable limelightTable;

  public Limelight() {
   
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  }

  /**
   * Returns a value of the offset on the x-axis of the camera to the target in degrees. Negative values mean the target is to the left of the camera
   */
  public double getXOffset() {
    return xOffset;
  }

  /**
   * Returns true if a target is detected
   */
  public boolean isTargetDetected() {
    return (targetValue > 0.0);
  }

  /**
   * Returns true if the target is within a range of the center crosshair of the camera
   */
  public boolean isTargetCentered() {
    return ((xOffset > -1.5) && (xOffset < 1.5) && (xOffset != 0.0));
  }

  /**
   *  Calculates the total angle by adding the mounting angle with the y-axis offset angle of the limelight in degrees
   */
  public double limelightAngle() {
    return (Constants.kLimelightAngle + yOffset);
  }

  /**
   * Chooses which pipeline to use on the limelight
   * 
   * @param pipeline Which pipeline to use on the limelight (0-9)
   */
  public void setPipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    limelightTable.getEntry("Pipeline").setValue(pipeline);
  }

  /**
   * Returns the value of the pipeline from the network table
   * 
   * @return pipelineValue
   */
  public double getPipeline() {
    NetworkTableEntry pipeline = limelightTable.getEntry("Pipeline");
    double pipelineValue = pipeline.getDouble(0.0);
    return pipelineValue;
  }

  public void updateLimelight() {
    // Updates the values of the limelight on the network table
    xOffset = limelightTable.getEntry("tx").getDouble(0.0);
    yOffset = limelightTable.getEntry("ty").getDouble(0.0);
    targetArea = limelightTable.getEntry("ta").getDouble(0.0);
    targetValue = limelightTable.getEntry("tv").getDouble(0.0);

  }

  public void log() {
    // Updates the SmartDashboard with limelight values
    SmartDashboard.putNumber("LimelightXOffset", xOffset);
    SmartDashboard.putNumber("LimelightYOffset", yOffset);
    SmartDashboard.putNumber("LimelightAreaPercentage", targetArea);
    SmartDashboard.putBoolean("Target Centered", isTargetCentered());
    SmartDashboard.putBoolean("Target Detected", isTargetDetected());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLimelight();
    log();
  }
}


