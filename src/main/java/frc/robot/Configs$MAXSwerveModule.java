// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs$MAXSwerveModule {
   public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
   public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

   public Configs$MAXSwerveModule() {
   }

   static {
      double drivingFactor = 0.05077956125529683;
      double turningFactor = 6.283185307179586;
      double drivingVelocityFeedForward = 0.2081708518741928;
      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig.encoder.positionConversionFactor(drivingFactor).velocityConversionFactor(drivingFactor / 60.0);
      drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0.0, 0.0).velocityFF(drivingVelocityFeedForward).outputRange(-1.0, 1.0);
      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig.absoluteEncoder.inverted(true).positionConversionFactor(turningFactor).velocityConversionFactor(turningFactor / 60.0);
      turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.0, 0.0, 0.0).outputRange(-1.0, 1.0).positionWrappingEnabled(true).positionWrappingInputRange(0.0, turningFactor);
   }
}
