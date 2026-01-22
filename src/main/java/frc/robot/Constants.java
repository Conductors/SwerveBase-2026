package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    //RoboRio Constants
    public static final double kDefaultPeriod = 0.02; //50Hz

    //Drivetrain (whole robot) constants
    public static final double kMaxRobotSpeed = 3; // meters per second
    public static final double kMaxRobotSpeedLowGear = 0.8;
    public static final double kMaxRobotAngularSpeed = 2*Math.PI; // 1/2 rotation per second = Pi
    public static final double kMaxRobotAngularSpeedLowGear = Math.PI; // 1/2 rotation per second = Pi
    public static final double kWheelRadius = 0.0508;   //4" mk4 wheels
    public static final double kGearRatio = 8.1;  //actual 8.1
    public static final int kEncoderResolution = 42;

    //Swerve Module Configs
    public static final double kRobotWidth  = 0.303;      //Distance in x-coor from center of robot to swerve module
    public static final double kRobotLength = 0.303;      //Distance in y-coor from center of robot to swerve module
    public static final double kTurnMotorMaxAngSpeed = 2*Math.PI*2;     //2*pi*2 = 2 rotations per second

    public static final int kFL_DriveChannel = 17;
    public static final int kFL_TurnChannel  = 16;
    public static final int kFL_TurnEncoderChannel = 3;
    public static final double kFL_TurnEncoderOffset = 0.837;

    public static final int kFR_DriveChannel = 13;
    public static final int kFR_TurnChannel  = 12;
    public static final int kFR_TurnEncoderChannel = 1;
    public static final double kFR_TurnEncoderOffset = 0.903;

    public static final int kBL_DriveChannel = 15;
    public static final int kBL_TurnChannel  = 14;
    public static final int kBL_TurnEncoderChannel = 2;
    public static final double kBL_TurnEncoderOffset = 0.335;

    public static final int kBR_DriveChannel = 11;
    public static final int kBR_TurnChannel  = 10;
    public static final int kBR_TurnEncoderChannel = 0;
    public static final double kBR_TurnEncoderOffset = 0.782;


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 30;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.3;
        public static final double kPYController = 1.3;
        public static final double kPThetaController = 1;
    
        public static final double kPP_PXController = 5;
        public static final double kPP_PYController = 1.3;
        public static final double kPP_PThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final String[] kAutoProgram = {"None", "Auto 1", "Auto 2", "Auto 3", "BackUp", "ScoreOneCoral", "ScoreCoralTake1Algae", "diagScoreReef", "ScoreCoralClearAlgae"};
      }

  public static final class AprilTagConstants {
    public static final int[] leftTags = {13, 3};      //red, blue alliance tags
    public static final int[] frontTags = {1, 11};     //red, blue alliance tags
    public static final int[] rightTags = {2, 12};     //red, blue alliance tags
  }

}