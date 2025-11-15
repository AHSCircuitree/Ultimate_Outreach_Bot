// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;
//
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants$DriveConstants {
   public static final double kMaxSpeedMetersPerSecond = 4.8;
   public static final double kMaxAngularSpeed = 6.283185307179586;
   public static final double kTrackWidth = Units.inchesToMeters(16.5);
   public static final double kWheelBase = Units.inchesToMeters(16.5);
   public static final SwerveDriveKinematics kDriveKinematics;
   public static final double kFrontLeftChassisAngularOffset = -1.5707963267948966;
   public static final double kFrontRightChassisAngularOffset = 0.0;
   public static final double kBackLeftChassisAngularOffset = Math.PI;
   public static final double kBackRightChassisAngularOffset = 1.5707963267948966;
   public static final int kFrontLeftDrivingCanId = 11;
   public static final int kRearLeftDrivingCanId = 13;
   public static final int kFrontRightDrivingCanId = 15;
   public static final int kRearRightDrivingCanId = 16;
   public static final int kFrontLeftTurningCanId = 10;
   public static final int kRearLeftTurningCanId = 12;
   public static final int kFrontRightTurningCanId = 14;
   public static final int kRearRightTurningCanId = 17;
   public static final boolean kGyroReversed = false;

   public Constants$DriveConstants() {
   }

   static {
      kDriveKinematics = new SwerveDriveKinematics(new Translation2d[]{new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)});
   }
}
