// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SlewRateLimiterEX {
  private double m_positiveRateLimit;
  private double m_negativeRateLimit;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveRateLimit The positive rate-of-change limit, in units per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The positive rate-of-change limit, in units per
   *     second. This is expected to be positive.
   * @param initialValue The initial value of the input.
   */
  public SlewRateLimiterEX(double positiveRateLimit, double negativeRateLimit, double initialValue) {
    m_positiveRateLimit = positiveRateLimit;
    m_negativeRateLimit = negativeRateLimit;
    m_prevVal = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param positiveRateLimit The positive rate-of-change limit, in units per second.
   * @param negativeRateLimit The negative rate-of-change limit, in units per second.
   */
  public SlewRateLimiterEX(double positiveRateLimit,double negativeRateLimit) {
    this(positiveRateLimit, negativeRateLimit, 0);
  }

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public SlewRateLimiterEX(double rateLimit) {
    this(rateLimit, rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    m_prevVal +=
        MathUtil.clamp(
            input - m_prevVal,
            -m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * sets the slew rate
   * 
   * @param positiveRate the positive rate to set
   * @param negativeRate the negative rate to set
   */
  public void setRate(double positiveRate,double negativeRate) {
    m_positiveRateLimit = positiveRate;
    m_negativeRateLimit = negativeRate;
  }

  /**
   * scales the previous value in the event that the max speed was changed
   * @param scalar
   */
  public void scalePrev(double scalar) {
    m_prevVal *= scalar;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }
}
