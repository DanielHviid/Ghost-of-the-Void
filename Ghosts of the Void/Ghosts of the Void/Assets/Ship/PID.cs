namespace controllerSystem
{
	public class PID
	{

		float gain, pGain, iGain, dGain;
		float maxIntegral;
		float integral = 0;
		float lastError = 0;

		public PID()
		{

		}

		public PID(float totalGain, float p, float i, float d, float maxInt)
		{
			gain = totalGain;
			pGain = p;
			iGain = i;
			dGain = d;
			maxIntegral = maxInt;
		}

		public float getOutput(float error, float deltaTime)
		{
			float result;
			integral += error * deltaTime;
			if (integral < -maxIntegral)
				integral = -maxIntegral;
			else if (integral > maxIntegral)
				integral = maxIntegral;
			result = gain * (pGain * error + iGain * integral + dGain * (error - lastError) / deltaTime);

			return result;

		}
	}
}
