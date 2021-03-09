/*
   PID calculation class
   by ; alshekly
   call .val and feed
             pid gains              (float p, float i, float d,
             maped radio input       float intended_Val,
             IMU axis angle          float actual_Val,
             Refresh rate            int period)
*/
class PID_Controls {
  public:
    //control
    double val (float p, float i, float d, float intended_Val, float actual_Val, int period) {

      float error = intended_Val - actual_Val;
      
      float pid_p = error * p;
      float pid_i = pid_i + (i * error);
      float pid_d = (d * ((error - old_Error) / period));

      float PID_out = pid_p + pid_i + pid_d;

      PID_out = map(PID_out, -150, 150, -500, 500);
      
      if (PID_out < -500) {
        PID_out = -500;
      }
      if (PID_out > 500) {
        PID_out = 500;
      }

      return PID_out;

      old_Error = error;
    }

};





/*
  PID Class
  a specific class for controlling UAV control_surfaces
  """

  """
  how to tune the PID values manually
    increase the d until the position (error) is stable (not changing a lot)
    increase the p slowly  until the error is almost to 0 (+or- 5ish%)
    increase the i by relatively very small increment until the error is 0

        # the PID values
    # proportional for the present error
    p = 8
    # integrator for the past error
    i = 0.1
    # derivative for the future error
    d = 15
*/
