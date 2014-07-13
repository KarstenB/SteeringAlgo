

public abstract class RoverAlgo {

	protected static final double M_PI_2 = Math.PI / 2;
	protected static final double robot_length = 720;
	protected static final double robot_width = 585;
	protected static final double HYPOT_ROVER = Math.hypot(robot_width, robot_length);
	protected static final int FRONTLEFT = 0;
	protected static final int REARLEFT = 1;
	protected static final int REARRIGHT = 2;
	protected static final int FRONTRIGHT = 3;

	public RoverAlgo() {
		super();

	}

	protected byte[] sendSpeed(int[] pwm_dir, int[] pwm_spd) {
		byte[] cmd = new byte[] { 0x7, 0x0, 16,//
				0, (byte) pwm_dir[0],//
				1, (byte) pwm_dir[1],//
				2, (byte) pwm_dir[2],//
				3, (byte) pwm_dir[3],//
				5, (byte) pwm_spd[0],//
				6, (byte) pwm_spd[1],//
				7, (byte) pwm_spd[2],//
				8, (byte) pwm_spd[3],//
		};
		return cmd;
	}

	public abstract byte[] compute(double x, double y, double ry);

	public abstract void setSpeedBoost(int progress);

}