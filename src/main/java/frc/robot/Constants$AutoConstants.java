// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants$AutoConstants {
   public static final double kMaxSpeedMetersPerSecond = 3.0;
   public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
   public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
   public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
   public static final double kPXController = 1.0;
   public static final double kPYController = 1.0;
   public static final double kPThetaController = 1.0;
   public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);

   public Constants$AutoConstants() {
   }
}
