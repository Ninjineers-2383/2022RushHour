package frc.robot.ninjaLib;

/**
 * MotionUtils provides convenience functions to convert between various units.
 * 
 * conventions:
 * 		"constants" that go into functions go at the end of the argument list
 * 		any encoder ratio calculations should be done at the first step (the conversion from native sensor units/ticks to rotations)
 * 
 */
public class MotionUtils {
	
	/**
	 * Convert Native Sensor Units/Ticks to Rotations
	 * @param ticks ticks from sensor
	 * @param ticksPerRotation ticks for 1 rotation (4096 for a mag encoder)
	 * @param encoderRatio if the sensor is behind a gear reduction, this is necessary to provide an accurate value
	 * @return sensor position in rotations
	 */
	public static double ticksToRotations(double ticks, double ticksPerRotation, double encoderRatio) {
		return (ticks / ticksPerRotation) * encoderRatio;
	}

	/**
	 * Convert rotations to real life units (feet or inches)
	 * @param circumference the circumference of the rotating object
	 * @return sensor distance in the units of @param circumference
	 */
	public static double rotationsToDistance(double rotations, double circumference) {
		return (rotations * circumference);
	}
	
	/**
	 * Convert distance in real life units (feet or inches) to rotations
	 * @param distance the distance traveled by the rotating object
	 * @param circumference the circumference of the rotating object
	 * @return sensor distance in the units of @param circumference
	 */
	public static double distanceToRotations(double distance, double circumference) {
		return (distance / circumference);
	}
	
	/**
	 * Convert Native Sensor Units/Ticks to RPS
	 * @param ticks velocity from sensor in native units (ticks per 100ms)
	 * @param ticksPerRotation ticks for 1 rotation (4096 for a mag encoder)
	 * @param period velocity sampling in period in seconds (should always be .1)
	 * @param encoderRatio if the sensor is behind a gear reduction, this is necessary to provide an accurate value
	 * @return sensor velocity in Rotations Per Second
	 */
	public static double ticksToRPS(double ticks, double ticksPerRotation, double period, double encoderRatio) {
		return ticks * ((1/period)/ticksPerRotation) * encoderRatio;
	}
	
	/**
	 * Convert Native Sensor Units/Ticks to RPM
	 * @param ticks velocity from sensor in native units (ticks per 100ms)
	 * @param ticksPerRotation ticks for 1 rotation (4096 for a mag encoder)
	 * @param encoderRatio if the sensor is behind a gear reduction, this is necessary to provide an accurate value
	 * @return sensor velocity in Rotations Per Second
	 */
	public static double ticksToRPM(double ticks, double ticksPerRotation, double period, double encoderRatio) {
		return ticksToRPS(ticks, ticksPerRotation, period, encoderRatio) * 60.0;
	}
}