package frc.robot.helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class SlewRateLimiter {
    private final double m_rateLimit;
    private double m_prevVal;
    private double m_prevTime;

    public SlewRateLimiter(double rateLimit, double initialValue) {
        m_rateLimit = rateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, 0);
    }

    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        if (input > m_prevVal && m_prevVal > 0 || input < m_prevVal && m_prevVal < 0) {
            m_prevVal += MathUtil.clamp(input - m_prevVal, 1.3 * -m_rateLimit * elapsedTime,
                    1.3 * m_rateLimit * elapsedTime);
        }
        m_prevVal += MathUtil.clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }
}
