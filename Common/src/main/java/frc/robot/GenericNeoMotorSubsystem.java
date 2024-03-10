// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public abstract class GenericNeoMotorSubsystem extends NinjasSubsytem {

  protected final GenericNeoMotorSubsystemConstants _constants;
  protected final CANSparkMax _master;
  protected final RelativeEncoder _relEncoder;
  protected final AbsoluteEncoder _absEncoder;
  protected final SparkPIDController _controller;
  protected final CANSparkMax[] _slaves;
  protected ControlState _controlState = ControlState.OPEN_LOOP;
  protected PeriodicIO _periodicIO = new PeriodicIO();
  protected GenericEntry position, velocity, setpoint, posError, velError, currentControl;

  /** Creates a new GenericMotorSubsystem. */
  protected GenericNeoMotorSubsystem(final GenericNeoMotorSubsystemConstants constants) {

    _constants = constants;
    _master = SparkMAXFactory.createDefaultSparkMax(_constants.kMasterConstants.id);
    _slaves = new CANSparkMax[_constants.kSlaveConstants.length];

    _relEncoder = _master.getEncoder();
    _absEncoder = _master.getAbsoluteEncoder();

    _master.setSoftLimit(SoftLimitDirection.kForward, _constants.kMaxUnitsLimit);
    _master.enableSoftLimit(SoftLimitDirection.kForward, true);

    _master.setSoftLimit(SoftLimitDirection.kReverse, _constants.kMinUnitsLimit);
    _master.enableSoftLimit(SoftLimitDirection.kReverse, true);

    _controller = _master.getPIDController();

    _controller.setP(_constants.kPositionKp, 0);
    _controller.setI(_constants.kPositionKi, 0);
    _controller.setIZone(_constants.kPositionKIzone, 0);
    _controller.setD(_constants.kPositionKp, 0);
    _master.burnFlash();

    for (int i = 0; i < _slaves.length; ++i) {
      _slaves[i] = SparkMAXFactory.createPermanentSlaveSparkMax(_constants.kSlaveConstants[i].id, _master,
          _constants.kSlaveConstants[i].invert);
      _slaves[i].burnFlash();
    }
    position = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("position", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    velocity = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("velocity", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    setpoint = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("setpoint", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    posError = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("posError", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    velError = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("velError", 0)
        .withWidget("Graph")
        .withSize(3, 3)
        .getEntry();
    currentControl = Shuffleboard.getTab(_constants.kSubsystemName)
        .add("currentControl", "")
        .withWidget("Text View")
        .withSize(3, 3)
        .getEntry();

    // Send a neutral command.
    stop();
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    if (_controlState == ControlState.MOTION_MAGIC) {
      // no motion magic for SParkmax
    } else if (_controlState == ControlState.POSITION_PID || _controlState == ControlState.MOTION_PROFILING) {
      _controller.setReference(_periodicIO.demand, ControlType.kPosition);
    } else {
      _master.set(_periodicIO.demand);
    }
  }

  public boolean atHomingLocation() {
    return false;
  }

  public synchronized String getControlState() {
    return _controlState.toString();
  }

  public void stop() {
    setOpenLoop(0.0);
    _master.stopMotor();
  }

  public synchronized void setSupplyCurrentLimit(int value) {
    _master.setSmartCurrentLimit(value);
  }

  public synchronized void setOpenLoop(double percentage) {
    if (_controlState != ControlState.OPEN_LOOP) {
      _controlState = ControlState.OPEN_LOOP;
    }
    _periodicIO.demand = percentage;
  }

  public synchronized double getVelError() {
    return _relEncoder.getVelocity() - _periodicIO.velocity_rps;
  }

  public synchronized double getSetpoint() {
    return (_controlState == ControlState.MOTION_MAGIC ||
        _controlState == ControlState.POSITION_PID ||
        _controlState == ControlState.MOTION_PROFILING) ? _periodicIO.demand : Double.NaN;
  }
  public synchronized double getPosition() {
    return _relEncoder.getPosition();
  }

  @Override
  public synchronized void outputTelemetry() {
    position.setDouble(getPosition());
    currentControl.setString(_controlState.toString());
    setpoint.setDouble(getSetpoint());
    velError.setDouble(getVelError());
    velError.setDouble(getVelError());

    // SmartDashboard.putNumber(_constants.kSubsystemName + " Current", _periodicIO.master_current);

  }

  // Recommend initializing in a static block!
  public static class SparkMaxConstants {
    public int id = -1;
    public boolean invert = false;

  }

  // Recommend initializing in a static block!
  public static class GenericNeoMotorSubsystemConstants {
    public String kSubsystemName = "ERROR_ASSIGN_A_NAME";

    public double kLooperDt = 0.01;
    public double kCANTimeout = 0.010; // use for important on the fly updates
    public int kLongCANTimeoutMs = 100; // use for constructors

    public SparkMaxConstants kMasterConstants = new SparkMaxConstants();
    public SparkMaxConstants[] kSlaveConstants = new SparkMaxConstants[0];

    public IdleMode kNeutralMode = IdleMode.kBrake;
    public double kHomePosition = 0.0; // Units
    public double kRotationsPerUnitDistance = 1.0;
    public double kSoftLimitDeadband = 0.0;

    public double kPositionKp = 0;
    public double kPositionKi = 0;
    public double kPositionKIzone = 0;
    public double kPositionKd = 0;
    public double kPositionKf = 0;
    public int kPositionDeadband = 0; // Ticks

    public double kVelocityFeedforward = 0;
    public double kArbitraryFeedforward = 0;
    public double kCruiseVelocity = 0; // Units/s
    public double kAcceleration = 0; // Units/s^2
    public double kJerk = 0; // Units/s^3
    public double kRampRate = 0.0; // s
    public double kMaxVoltage = 12.0;

    public int kSupplyCurrentLimit = 60; // amps
    public boolean kEnableSupplyCurrentLimit = false;

    public int kStatorCurrentLimit = 40; // amps
    public boolean kEnableStatorCurrentLimit = false;

    public float kMaxUnitsLimit = Float.POSITIVE_INFINITY;
    public float kMinUnitsLimit = Float.NEGATIVE_INFINITY;

    public int kStatusFrame8UpdateRate = 1000;
  }

  public static class PeriodicIO {
    // INPUTS
    public double timestamp;
    public double position_rots; // motor rotations
    public double position_units;
    public double velocity_rps;
    public double prev_vel_rps;
    public double output_percent;
    public double output_voltage;
    public double master_current;
    public double error_rotations;
    public boolean reset_occured;
    public double active_trajectory_position;
    public double active_trajectory_velocity;
    public double active_trajectory_acceleration;

    // OUTPUTS
    public double demand; // position (motor rots) or percent output
    public double feedforward;
  }

  protected enum ControlState {
    OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
  }
}
