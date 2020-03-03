package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {


  double validTarget;  // Whether the limelight has any valid targets (0 or 1)
  double xOffset;      // Horiszontal Offset from crosshair to target (-27 degrees to 27 degrees)
  double yOffset;      // Vertical Offset from crosshair to target (-20.5 degrees to 20.5 degrees)
  double targetArea;   // target area (0% of image to 100%)
  double skew;         // skew or rotation (-90 degrees to 0 degrees)
  double pipeLatency;  // the pipeline's latency contribution (ms) add at least 11 ms for image capture latency
  double tshort;       // Sidelength of shortest side of the fitted bounding box (pixels)
  double tlong;        // Sidelength of longest side of the fitted bounding box (pixels)
  double thor;         // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  double tvert;        // Vertical sidelength of the rough bounding box (0 - 320 pixels)
  double getpipe;      // True active pipeline index of the camera (0 .. 9)
  double camtran;      // Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)

  boolean lightOn = true;
      // below group is used for generating drive and steer from vision
  double LimelightDriveCommand;
  double LimelightDriveMax = 0.4;
  double LimelightSteerCommand;
  // double Drive_K = 0.03;  // tune. Constant for generating drive speed from vision
  // double Drive_D = 0.18;     // tune. Constant for generating drive speed from area errors in vision
  double Drive_K = 0.0;  // tune. Constant for generating drive speed from vision
  double Drive_D = 0;  
  // double Steer_K = 0.075;  // tune. Constant for generating turn speed from vision. (OG) P: 0.07 D: 0.2
  // double Steer_D = 0.3;
  double Steer_K = 0.06;  // tune. Constant for generating turn speed from vision. (OG) P: 0.07 D: 0.2
  double Steer_D = 0.2;
  double Skew_P = 0.0;
  double txError = 0;
  double skewError = 0;
  double prevSkew = 0;
  double previoustxError = 0;
  double deltatxError = 0;
  double DesiredTargetArea = 7.67;  // this needs to be tuned to robot
  double areaError = 0;   // just set to zero to start with 
  double previousAreaError = 0; // same stroy as above
  double deltaError = 0; // same story as above
  double desiredSkew = 0;

 
  
  
  
  public double GenerateDrive() {
    areaError = DesiredTargetArea - getTargetArea();
    if (areaError != previousAreaError){
      deltaError = areaError - previousAreaError;
    }
    if (!isTargetValid()) {
      LimelightDriveCommand = 0.0;
    } else {
      LimelightDriveCommand = (areaError * Drive_K) + (deltaError * Drive_D);
    }

    if (LimelightDriveCommand > LimelightDriveMax) {
      LimelightDriveCommand = LimelightDriveMax;
    }
    previousAreaError = areaError;
    return LimelightDriveCommand;
  }

  public double GenerateSteer() {
    txError = getXOffset();                              // I know that this may be slightly redundent but fight me!
    if (txError != previoustxError){
      deltatxError = txError - previoustxError;
    }
    if (!isTargetValid()) {
      LimelightSteerCommand = 0.0;
    } else {
      LimelightSteerCommand = (txError * Steer_K) + (deltatxError * Steer_D);
    }
    previoustxError = txError;
    return LimelightSteerCommand;
  }



  public boolean fetchUpdate() {
    try {
      validTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      yOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    } catch(Exception ex) {
      return false;
    }
    return true;
  }


  public boolean isTargetValid() { 
    if (validTarget > 0.5) 
    {
      return true;
    } 
    else
    {
      return false;
    }
  }

  
  public void getRawSkew() {
    System.out.println(skew); 
  }
  
  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getTargetArea() {
    return targetArea;
  }




  public void setLedMode(int ledMode) { // 0-use the LED mode set in the current pipeline   1-force off   2-force blink  3-force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }

  public double getLedMode() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0);
  }

  public void setCamMode(int camMode) { // 0-Vision processor   1-DriverCamera
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
  }

  public double getCamMode () {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0);
  }

  public void setPipeLine(int pipeLine) { // 0 through 9 as pipeline
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeLine);
  }

    // (stream modes) 0 - Standard: Side-by-side streams if a webcam is attached to Limelight
    // 1 - Pip Main: the secondary camara stream is placed in the lower-right corner of the primary camera stream
    // 2 - PiP Secondary: The primary camera stream is placed in the lower-right corner of the secondary camera stream
  public void setStream(int streamMode) { 
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(streamMode);
  }


  public void PipelineOnPress(boolean button) {
    if (button && (getCamMode() > 0.5)) {
      setCamMode(0);
    } else if (!button && (getCamMode() < 0.5)) {
      setCamMode(1);
    }
  }

  public void turnOnLight(boolean button) {
    if (button && (getLedMode() > 0.5)) {
      setLedMode(0);
    // } else if (!button && getLedMode() < 0.5) {
    //   setLedMode(1);
    }
  }

  public void lightToggle(boolean button) {
    if(button) {
      if(lightOn) {
        lightOn = false;
      } else {
        lightOn = true;
      }
    }
    if(lightOn && (getLedMode() > 0.5)) {
      setLedMode(0);
    } 
    else if(!lightOn && (getLedMode() < 0.5)) {
      setLedMode(1);
    }
  }


}