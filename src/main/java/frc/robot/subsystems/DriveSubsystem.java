// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants$DriveConstants;

public class DriveSubsystem extends SubsystemBase {
   private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(11, 10, -1.5707963267948966);
   private final MAXSwerveModule m_frontRight = new MAXSwerveModule(15, 14, 0.0);
   private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(13, 12, Math.PI);
   private final MAXSwerveModule m_rearRight = new MAXSwerveModule(16, 17, 1.5707963267948966);
   private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
   SwerveDriveOdometry m_odometry;

   public DriveSubsystem() {
      this.m_odometry = new SwerveDriveOdometry(Constants$DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(this.m_gyro.getAngle(IMUAxis.kZ)), new SwerveModulePosition[]{this.m_frontLeft.getPosition(), this.m_frontRight.getPosition(), this.m_rearLeft.getPosition(), this.m_rearRight.getPosition()});
      HAL.report(31, 17);
   }

   public void periodic() {
      this.m_odometry.update(Rotation2d.fromDegrees(this.m_gyro.getAngle(IMUAxis.kZ)), new SwerveModulePosition[]{this.m_frontLeft.getPosition(), this.m_frontRight.getPosition(), this.m_rearLeft.getPosition(), this.m_rearRight.getPosition()});
   }

   public Pose2d getPose() {
      return this.m_odometry.getPoseMeters();
   }

   public void resetOdometry(Pose2d pose) {
      this.m_odometry.resetPosition(Rotation2d.fromDegrees(this.m_gyro.getAngle(IMUAxis.kZ)), new SwerveModulePosition[]{this.m_frontLeft.getPosition(), this.m_frontRight.getPosition(), this.m_rearLeft.getPosition(), this.m_rearRight.getPosition()}, pose);
   }

   public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      double xSpeedDelivered = xSpeed * 4.8;
      double ySpeedDelivered = ySpeed * 4.8;
      double rotDelivered = rot * 6.283185307179586;
      SwerveModuleState[] swerveModuleStates = Constants$DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(this.m_gyro.getAngle(IMUAxis.kZ))) : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.8);
      this.m_frontLeft.setDesiredState(swerveModuleStates[0]);
      this.m_frontRight.setDesiredState(swerveModuleStates[1]);
      this.m_rearLeft.setDesiredState(swerveModuleStates[2]);
      this.m_rearRight.setDesiredState(swerveModuleStates[3]);
   }

   public void setX() {
      this.m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
      this.m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
      this.m_rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
      this.m_rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
   }

   public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.8);
      this.m_frontLeft.setDesiredState(desiredStates[0]);
      this.m_frontRight.setDesiredState(desiredStates[1]);
      this.m_rearLeft.setDesiredState(desiredStates[2]);
      this.m_rearRight.setDesiredState(desiredStates[3]);
   }

   public void resetEncoders() {
      this.m_frontLeft.resetEncoders();
      this.m_rearLeft.resetEncoders();
      this.m_frontRight.resetEncoders();
      this.m_rearRight.resetEncoders();
   }

   public void zeroHeading() {
      this.m_gyro.reset();
   }

   public double getHeading() {
      return Rotation2d.fromDegrees(this.m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
   }

   public double getTurnRate() {
      return this.m_gyro.getRate(IMUAxis.kZ) * 1.0;
   }
}
