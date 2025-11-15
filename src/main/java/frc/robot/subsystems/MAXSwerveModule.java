// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MAXSwerveModule {
   private final SparkMax m_drivingSpark;
   private final SparkMax m_turningSpark;
   private final RelativeEncoder m_drivingEncoder;
   private final AbsoluteEncoder m_turningEncoder;
   private final SparkClosedLoopController m_drivingClosedLoopController;
   private final SparkClosedLoopController m_turningClosedLoopController;
   private double m_chassisAngularOffset = 0.0;
   private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
//
   public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
      this.m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
      this.m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
      this.m_drivingEncoder = this.m_drivingSpark.getEncoder();
      this.m_turningEncoder = this.m_turningSpark.getAbsoluteEncoder();
      this.m_drivingClosedLoopController = this.m_drivingSpark.getClosedLoopController();
      this.m_turningClosedLoopController = this.m_turningSpark.getClosedLoopController();
      this.m_drivingSpark.configure(frc.robot.Configs$MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      this.m_turningSpark.configure(frc.robot.Configs$MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      this.m_chassisAngularOffset = chassisAngularOffset;
      this.m_desiredState.angle = new Rotation2d(this.m_turningEncoder.getPosition());
      this.m_drivingEncoder.setPosition(0.0);
   }

   public SwerveModuleState getState() {
      return new SwerveModuleState(this.m_drivingEncoder.getVelocity(), new Rotation2d(this.m_turningEncoder.getPosition() - this.m_chassisAngularOffset));
   }

   public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(this.m_drivingEncoder.getPosition(), new Rotation2d(this.m_turningEncoder.getPosition() - this.m_chassisAngularOffset));
   }

   public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(this.m_chassisAngularOffset));
      correctedDesiredState.optimize(new Rotation2d(this.m_turningEncoder.getPosition()));
      this.m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
      this.m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
      this.m_desiredState = desiredState;
   }

   public void resetEncoders() {
      this.m_drivingEncoder.setPosition(0.0);
   }
}
