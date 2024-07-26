// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

public class GenericSparkMaxSubsystem extends NinjaMotorController {

  protected final CANSparkMax _master;
  protected final CANSparkMax[] _slaves;

  protected final RelativeEncoder _relEncoder;
  // protected final AbsoluteEncoder _absEncoder;
  protected final SparkPIDController _controller;

  private final TrapezoidProfile _profile;
  private final Timer trapozoidTimer = new Timer();
  
  /** Creates a new GenericMotorSubsystem. */
  public GenericSparkMaxSubsystem(final NinjaMotorSubsystemConstants constants) {
    super(constants);

    _constants = constants;
    _master = SparkMAXFactory.createDefaultSparkMax(_constants.kMasterConstants.id);
    _slaves = new CANSparkMax[_constants.kSlaveConstants.length];

    _relEncoder = _master.getEncoder();
    // _absEncoder = _master.getAbsoluteEncoder(Spark);

    _profile = new TrapezoidProfile(new Constraints(_constants.kCruiseVelocity, _constants.kAcceleration));
    _controller = _master.getPIDController();
    _relEncoder.setPositionConversionFactor(constants.kGearRatio);
    _relEncoder.setVelocityConversionFactor(constants.kGearRatio/60);
    _master.burnFlash();

    for (int i = 0; i < _slaves.length; ++i) {
      _slaves[i] = SparkMAXFactory.createPermanentSlaveSparkMax(_constants.kSlaveConstants[i].id, _master,
          _constants.kSlaveConstants[i].invert);
      _slaves[i].burnFlash();
    }

    // Send a neutral command.
    stop();
  }

  /**
   * Periodically writing commands and control effort to this motor group.
   * this method is called in the periodic method of the NinjaSubsystem
   */
  @Override
  public synchronized void writePeriodicOutputs() {
    if (_controlState == ControlState.MOTION_MAGIC) {
      _controller.setReference(_profile.calculate(trapozoidTimer.get(), new State(getPosition(), 0), new State(demand, 0)).position, ControlType.kPosition);
    } else if (_controlState == ControlState.POSITION_PID || _controlState == ControlState.MOTION_PROFILING) {
      _controller.setReference(demand, ControlType.kPosition);
    } else {
      _master.set(demand);
    }

  }
  /**
   * Sets the forward soft limit for this motor group
   * @param sofLimit position of forward soft limit
   */
  @Override
  public void setForwardSoftLimit(double sofLimit) {
    _master.setSoftLimit(SoftLimitDirection.kForward, (float)sofLimit);
    _master.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

   /*
   * Sets the reverse soft limit for this motor group
   * @param sofLimit position of forward soft limit
   */
  @Override
  public void setReverseSoftLimit(double sofLimit) {
    _master.setSoftLimit(SoftLimitDirection.kReverse, (float)sofLimit);
    _master.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  /**
   * Checks if arrived at wanted position
   * @return if error of demand (wanted position) and current position is 0
   */
  @Override
  public boolean atHomingLocation() {
    return getPosition() - demand == 0;
  }

  /**
   * Returns the state of the current motor group
   * @return current motor gorup state
   */
  public synchronized String getControlState() {
    return _controlState.toString();
  }

  /**
   * Stops this motor group
   */
  @Override
  public void stop() {
    set(0.0);
    _master.stopMotor();
  }

  /**
   * Sets the limit to the possible suppliable current to this motor group
   * 
   * @param value limit of supply current
   */
  public synchronized void setSupplyCurrentLimit(int value) {
    _master.setSmartCurrentLimit(value);
  }

  /**
   * Set method for this motor group for a percentage demand
   * 
   * @param percentage Amount of demand in percentage to apply
   */
  @Override
  public void set(double percentage) {
    if (_controlState != ControlState.OPEN_LOOP) {
      _controlState = ControlState.OPEN_LOOP;
    }
    demand = percentage;
  }

  /**
   * Set method for this motor group for a position through a motion profile demand
   * 
   * @param pos demanded position to reach
   */
  @Override
  public void set(State pos) {
    if (_controlState != ControlState.MOTION_MAGIC) {
      _controlState = ControlState.MOTION_MAGIC;
    }
    trapozoidTimer.restart();
    demand = pos.position;
  }

  /**
   * Returns the error of the velocity (works only when demand is set to velocity)
   * 
   * @return Velocity error
   */
  public synchronized double getVelError() {
    return _relEncoder.getVelocity() - demand;
  }

  /**
   * Returns the current demanded setpoint if state is set to either: MOTION_MAGIC, POSITION_PID, MOTION_PROFILING.
   * 
   * @return Current demanded setpoin
   */
  public synchronized double getSetpoint() {
    return (_controlState == ControlState.MOTION_MAGIC ||
        _controlState == ControlState.POSITION_PID ||
        _controlState == ControlState.MOTION_PROFILING) ? demand : Double.NaN;
  }

  /**
   * Returns current position of sparkMAX relative encoder
   * 
   * @return Current position
   */
  @Override
  public synchronized double getPosition() {
    return _relEncoder.getPosition();
  }

  /**
   * Returns current D constant of this motor group
   * 
   * @return D constant of this motor group
   */
  @Override
  public double getD() {
    return _controller.getD();
  }

  /**
   * Returns current I constant of this motor group
   * 
   * @return I constant of this motor group
   */
  @Override
  public double getI() {
    return _controller.getI();
  }

  /**
   * Returns current I ZONE constant of this motor group
   * 
   * @return I ZONE constant of this motor group
   */
  @Override
  public double getIzone() {
    return _controller.getIZone();
  }

  /**
   * Returns current P constant of this motor group
   * 
   * @return P constant of this motor group
   */
  @Override
  public double getP() {
    return _controller.getP();
  }

  /**
   * Returns current speed of the master of this motor group
   * 
   * @return Speed of the master of the master of this motor group
   */
  @Override
  public double get() {
    return _master.get();
  }

  /**
   * Sets the PID constants for this motor group
   * 
   * @param Kp 
   * @param Ki
   * @param Kd
   * @param KIzone
   */
  @Override
  public void setPIDconstants(double Kp, double Ki, double Kd, double KIzone) {
    _controller.setP(Kp, 0);
    _controller.setI(Ki, 0);
    _controller.setIZone(KIzone, 0);
    _controller.setD(Kd, 0);
  }

  /**
   * Sets the position of this relative encoder of the master of this motor group
   * 
   * @param pos Wanted position 
   */
  @Override
  public void setPosition(double pos) {
    _relEncoder.setPosition(pos);
  }

  /**
   * Home a position using regular PID control loop
   * 
   * @param pos Demanded positiom
   */
  @Override
  public void setHomingPosition(double pos) {
    if (_controlState != ControlState.POSITION_PID) {
      _controlState = ControlState.POSITION_PID;
    }
    demand = pos;
  }

  public static GenericSparkMaxSubsystem createSparkMaxMotorGroup(final NinjaMotorSubsystemConstants constants){
    return new GenericSparkMaxSubsystem(constants);
  }

  
  
}
