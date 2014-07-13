public class VectorAlgo extends RoverAlgo {

	private static final double M_PI = Math.PI;
	protected final Wheel wheelValues[] = new Wheel[4];
	private int speedFac = 127;

	protected class Wheel {
		double x;
		double y;
		double orientation;
		double speed;

		@Override
		public String toString() {
			return "Wheel [x=" + x + ", y=" + y + ", orientation=" + orientation + ", speed=" + speed + "]";
		}
	}

	public VectorAlgo() {
		super();
		for (int i = 0; i < wheelValues.length; i++) {
			wheelValues[i] = new Wheel();
		}
	}

	private static double lastWheelOrientations[] = new double[4];

	@Override
	public byte[] compute(double y, double x, double ry) {

		// contains vectorized speeds and servo values for each wheel

		int[] pwm_dir = new int[4];
		int[] pwm_spd = new int[4];

		// calculate rotation component of the wheel's speeds
		double rfac = ry / HYPOT_ROVER;
		wheelValues[FRONTLEFT].x = -robot_width * rfac;
		wheelValues[FRONTLEFT].y = robot_length * rfac;
		wheelValues[REARLEFT].x = -robot_width * rfac;
		wheelValues[REARLEFT].y = -robot_length * rfac;
		wheelValues[REARRIGHT].x = robot_width * rfac;
		wheelValues[REARRIGHT].y = -robot_length * rfac;
		wheelValues[FRONTRIGHT].x = robot_width * rfac;
		wheelValues[FRONTRIGHT].y = robot_length * rfac;

		double maxspeed = 0.0;
		// add the linear movement vector
		for (int i = 0; i < 4; ++i) {
			wheelValues[i].x += x;
			wheelValues[i].y -= y;
			maxspeed = Math.max(maxspeed, Math.hypot(wheelValues[i].x, wheelValues[i].y));
		}
		for (int i = 0; i < 4; ++i) {
			// normalize all speeds to [-1..1]: Divide by the largest speed if
			// it is >1.0
			if (maxspeed > 1.0) {
				wheelValues[i].y /= maxspeed;
				wheelValues[i].x /= maxspeed;
			}

			// now the wheel speed vectors are finished.
			// calculate angles and absolute speeds.
			wheelValues[i].orientation = Math.atan2(wheelValues[i].y, wheelValues[i].x);
			wheelValues[i].speed = Math.hypot(wheelValues[i].x, wheelValues[i].y);
			// angles are limited => reverse speed if necessary
			if (wheelValues[i].orientation < -M_PI_2) {
				wheelValues[i].orientation += M_PI;
				wheelValues[i].speed *= -1.0;
			}
			if (wheelValues[i].orientation > M_PI_2) {
				wheelValues[i].orientation -= M_PI;
				wheelValues[i].speed *= -1.0;
			}

			// calculate alternative orientation
			double orientation2;
			if (wheelValues[i].orientation < 0) {
				orientation2 = (wheelValues[i].orientation + M_PI);
			} else {
				orientation2 = (wheelValues[i].orientation - M_PI);
			}
			double orientationDelta1 = lastWheelOrientations[i] - wheelValues[i].orientation;
			double orientationDelta2 = lastWheelOrientations[i] - orientation2;
			if (abs(orientationDelta2) < abs(orientationDelta1)) {
				wheelValues[i].orientation = orientation2;
				wheelValues[i].speed *= -1.0;
			}
			if (wheelValues[i].orientation < (-M_PI_2 - 1)) {
				wheelValues[i].orientation += M_PI;
				wheelValues[i].speed *= -1.0;
			}
			if (wheelValues[i].orientation > (M_PI_2 + 1)) {
				wheelValues[i].orientation -= M_PI;
				wheelValues[i].speed *= -1.0;
			}
			lastWheelOrientations[i] = wheelValues[i].orientation;
			// calculate steering servo value
			if (wheelValues[i].orientation < 0.0) {
				pwm_dir[i] = (int) (0x80 + ((wheelValues[i].orientation * (0x80 - 0xFF)) / M_PI_2));
			} else {
				pwm_dir[i] = (int) (0x80 + ((wheelValues[i].orientation * (0 - 0x80)) / M_PI_2));
			}

			// calculate driving servo value
			if (i > 1)
				pwm_spd[i] = (int) (0x80 + (-speedFac * wheelValues[i].speed));
			else
				pwm_spd[i] = (int) (0x80 + (speedFac * wheelValues[i].speed));

		}
		return sendSpeed(pwm_dir, pwm_spd);
	}

	private double abs(double orientationDelta2) {
		return Math.abs(orientationDelta2);
	}

	public static void main(String[] args) {
		VectorAlgo v = new VectorAlgo();
		for (int i = 0; i < 628; i++) {
			v.compute(0, 0.5, i / 100.);
			printWheels(v);
			v.compute(0, 0.5, i / 100.);
			printWheels(v);
		}

		// 00 4f 01 4f 02 51 03 4f 05 bc 06 bc 07 44 08 44
		// printHex(v.compute(0.665, 0.930, 0.0));
		// 00 db 01 db 02 d9 03 db 05 ab 06 ab 07 54 08 54
	}

	private static void printWheels(VectorAlgo v) {
		for (int j = 0; j < 4; j++)
			System.out.printf(" W:%d %5.3f", j, v.wheelValues[j].orientation);
		System.out.println();
	}

	@Override
	public void setSpeedBoost(int speed) {
		speedFac = speed;
	}
}
