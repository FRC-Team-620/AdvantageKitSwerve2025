package org.jmhsrobotics.frc2025.subsystems.drive.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2025.subsystems.drive.DriveConstants.ThriftyConstants;
import org.jmhsrobotics.frc2025.subsystems.drive.SparkOdometryThread;
import org.jmhsrobotics.frc2025.util.SparkUtil;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOThrifty implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOThrifty(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> DriveConstants.frontLeftZeroRotation;
          case 1 -> DriveConstants.frontRightZeroRotation;
          case 2 -> DriveConstants.backLeftZeroRotation;
          case 3 -> DriveConstants.backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> ThriftyConstants.frontLeftDriveCanId;
              case 1 -> ThriftyConstants.frontRightDriveCanId;
              case 2 -> ThriftyConstants.backLeftDriveCanId;
              case 3 -> ThriftyConstants.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> ThriftyConstants.frontLeftTurnCanId;
              case 1 -> ThriftyConstants.frontRightTurnCanId;
              case 2 -> ThriftyConstants.backLeftTurnCanId;
              case 3 -> ThriftyConstants.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ThriftyConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(ThriftyConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(ThriftyConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            ThriftyConstants.driveKp, 0.0,
            ThriftyConstants.driveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(ThriftyConstants.turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ThriftyConstants.turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(ThriftyConstants.turnEncoderInverted)
        .positionConversionFactor(ThriftyConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(ThriftyConstants.turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            ThriftyConstants.turnPIDMinInput, ThriftyConstants.turnPIDMaxInput)
        .pidf(ThriftyConstants.turnKp, 0.0, ThriftyConstants.turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    SparkUtil.ifOk(
        driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    SparkUtil.ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update turn inputs
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    SparkUtil.ifOk(
        turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    SparkUtil.ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        ThriftyConstants.driveKs * Math.signum(velocityRadPerSec)
            + ThriftyConstants.driveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            ThriftyConstants.turnPIDMinInput,
            ThriftyConstants.turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }
}
