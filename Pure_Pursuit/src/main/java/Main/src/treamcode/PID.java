package Main.src.treamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PID
{

    ElapsedTime timer = new ElapsedTime();
    double Kp;
    double Ki;
    double Kd;
    double integralSumLimit; // max integral value, should have its own constructor variable to change
    double lastError;
    double integralSum;
    double error;
    double derivative;
    double output;
    double Kf;


    public PID(double Kp, double Ki, double Kd, double integralSumLimit, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.integralSumLimit = integralSumLimit;
        this.Kf = Kf;
    }

    public double update(double target, double state, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output
        error = target-state;
        derivative = (error-lastError)/timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * target);
        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        }
        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }

    public double updateWithError(double errorHeading, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output
        error = errorHeading;
        derivative = (error-lastError)/timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        } else if (output < -maxOutput){
            output = -maxOutput; // accounts for negative as well
        }

        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }

    public double returnError(){
        return error;
    }
    public double returnOutput () {return output;}

    public double returnIntegralSum() {
        return integralSum;
    }

}
