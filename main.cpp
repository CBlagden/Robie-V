
#define PI 3.1415926

vex::brain Brain;

vex::motor BackRightDrive = vex::motor(vex::PORT2,vex::gearSetting::ratio18_1,true);
vex::motor FrontRightDrive = vex::motor(vex::PORT1,vex::gearSetting::ratio18_1,true);
vex::motor BackLeftDrive = vex::motor(vex::PORT9,vex::gearSetting::ratio18_1,false);
vex::motor FrontLeftDrive = vex::motor(vex::PORT10,vex::gearSetting::ratio18_1,false);


vex::motor ShooterRight = vex::motor(vex::PORT3,vex::gearSetting::ratio36_1,false);
vex::motor ShooterLeft = vex::motor(vex::PORT6,vex::gearSetting::ratio36_1,true);

vex::motor Arm = vex::motor(vex::PORT7,vex::gearSetting::ratio18_1,false);
vex::motor Intake = vex::motor(vex::PORT8,vex::gearSetting::ratio36_1,false);

vex::controller Controller = vex::controller();

void setLeftDrive(double power);
void setRightDrive(double power);

class PIDControllerWithFeedForward {
private:
    double kP;
    double kI;
    double kD;
    double kF;
    double integral;
    double prevError;
    
public:
    PIDControllerWithFeedForward(double kP, double kI, double kD, double kF) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
        this->kF = kF;
    }
    
    double update(double curVal, double goal) {
        double error = goal - curVal;
        
        double proportional = kP * error;
        integral += kI * (integral + error);
        double derivative = kD * (prevError - error);
        double feedForward = kF * goal; 
        
        prevError = error;
        return proportional + integral + derivative + feedForward;
    }
    
    double getError() {
        return prevError;
    }
    
    void setConstants(double p, double i, double d, double f) {
        kP = p; kI = i; kD = d; kF = f;
    }
        
};

class PIDController {
private:
    double kP;
    double kI;
    double kD;
    double integral;
    double prevError;
    
public:
    PIDController(double kP, double kI, double kD) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }
    
    double update(double curVal, double goal) {
        double error = goal - curVal;
        
        double proportional = kP * error;
        integral += kI * (integral + error);
        double derivative = kD * (prevError - error);
        
        prevError = error;
        return proportional + integral + derivative;
    }
    
    double getError() {
        return prevError;
    }
        
};

double runShooter(double goal, double p, double i, double d, double f) {
    PIDControllerWithFeedForward pid(p, i, d, f);
    int n = 100;
    double err = 0.0;
    for (int i = 0; i < 2*n; i++) {
        double shooterSpeed = (ShooterRight.velocity(vex::velocityUnits::rpm) + ShooterLeft.velocity(vex::velocityUnits::rpm)) / 2.0;
        double power = pid.update(shooterSpeed, goal);
        
        if (i >= n) {
          err += pid.getError();
        }
        
        ShooterRight.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
        ShooterLeft.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
    }
    return err / (float)n;
}

double sum(double arr[], int size) {
    double s = 0.0;
    for (int i = 0; i < size; i++) {
        s += arr[i];
    }
    return s;
}

void twiddle(double tol, double goal) {
    double p[4] = {0.0, 0.0, 0.0, 0.0};
    double dp[4] = {1.0, 1.0, 1.0, 1.0};
    double bestErr = 2000 * 100;
    while (sum(dp, 4) > tol) {
        for (int i = 0; i < 4; i++) {
            p[i] += dp[i];
            double err = runShooter(goal, p[0], p[1], p[2], p[3]);
            if (err < bestErr) {
                bestErr = err;
                dp[i] *= 1.1;
            } else {
                p[i] -= 2 * dp[i];
                double err = runShooter(goal, p[0], p[1], p[2], p[3]);
                if (err < bestErr) {
                    bestErr = err;
                    dp[i] *= 1.1;
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
    }
}

double limit(double v, double limit) {
	return (abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
}

double pastLeftPower = 0;
double pastRightPower = 0;
double quickStopAccumulator = 0;
double oldWheel = 0;

void chezyDrive(double throttle, double wheel, bool isQuickTurn) {
	double wheelNonLinearity;
    
    throttle = throttle / 127.0;
    wheel = wheel / 127.0;

	double negInertia = wheel - oldWheel;
	oldWheel = wheel;
	//writeDebugStreamLine("negInertia %4.4f", negInertia);
	wheelNonLinearity = 0.6;
	// Apply a sin function that's scaled to make it feel better.
	wheel = sin(PI / 2.0 * wheelNonLinearity * wheel)
	/ sin(PI / 2.0 * wheelNonLinearity);
	wheel = sin(PI / 2.0 * wheelNonLinearity * wheel)
	/ sin(PI / 2.0 * wheelNonLinearity);
	wheel = sin(PI / 2.0 * wheelNonLinearity * wheel)
	/ sin(PI / 2.0 * wheelNonLinearity);
	// writeDebugStreamLine("newWheel (after sin) %4.4f", wheel);

	double leftPwm, rightPwm, overPower;
	double sensitivity;

	double angularPower;
	double linearPower;

	// Negative inertia!
	double negInertiaAccumulator = 0.0;
	double negInertiaScalar;

	if (wheel * negInertia > 0.0) {
		negInertiaScalar = 1.5;
		} else {
		if (abs(wheel) > 0.65) {
			negInertiaScalar = 5.0;
			} else {
			negInertiaScalar = 3.0;
		}
	}
	sensitivity = 0.7;       // .9

	double negInertiaPower = negInertia * negInertiaScalar;
	negInertiaAccumulator += negInertiaPower;
	//	writeDebugStreamLine("negAccumulatotr %4.4f", negInertiaAccumulator);
	wheel = wheel + negInertiaAccumulator;
	if (negInertiaAccumulator > 1.0) {
		negInertiaAccumulator -= 1.0;
		} else if (negInertiaAccumulator < -1.0) {
		negInertiaAccumulator += 1.0;
		} else {
		negInertiaAccumulator = 0.0;
	}
	linearPower = throttle;

	// Quickturn!
	if (isQuickTurn) {
		// if (abs(linearPower) < 0.2) {
		double alpha = 0.2;
		quickStopAccumulator = (1.0 - alpha) * quickStopAccumulator + alpha
		* limit(wheel, 1.0) * 5.0;
		// }
		overPower = 1.0;
		sensitivity = 1.0;
		angularPower = wheel;
		} else {
		overPower = 0.0;
		angularPower = abs(throttle) * (wheel * 1.1) * sensitivity - quickStopAccumulator;
		if (abs(throttle) < .1) {
			angularPower = wheel;
		}
		if (quickStopAccumulator > 1.0) {
			quickStopAccumulator -= 1.0;
			} else if (quickStopAccumulator < -1.0) {
			quickStopAccumulator += 1.0;
			} else {
			quickStopAccumulator = 0.0;
		}
	}
	//float CONSTANT = 20;
	rightPwm = leftPwm = linearPower;
	leftPwm += angularPower;
	rightPwm -= angularPower;


	pastRightPower = rightPwm;
	pastLeftPower = leftPwm;
	if (leftPwm > 1.0) {
		rightPwm -= overPower * (leftPwm - 1.0);
		leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
		leftPwm -= overPower * (rightPwm - 1.0);
		rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
		rightPwm += overPower * (-1.0 - leftPwm);
		leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
		leftPwm += overPower * (-1.0 - rightPwm);
		rightPwm = -1.0;
	}
	double leftPwmPower = leftPwm * 127.0;
	double rightPwmPower = rightPwm * 127.0;

	setLeftDrive(leftPwmPower);
	setRightDrive(rightPwmPower);
}

void arcadeDrive(double throttle, double turn) {
    setLeftDrive(throttle + turn);
    setRightDrive(throttle - turn);
}

void runDrive() {
    double throttle =  Controller.Axis3.value();
    double turn = Controller.Axis1.value();
    //chezyDrive(throttle, turn, false);
    arcadeDrive(throttle, turn);
}

double backPower = 0.0;

void runArm() {
       
      if (Controller.ButtonX.pressing()) {
          Arm.rotateTo(-870,vex::rotationUnits::deg, 100, vex :: velocityUnits :: pct, false);
      } else if (Controller.ButtonB.pressing()) {
          Arm.rotateTo(110, vex::rotationUnits::deg, 100, vex :: velocityUnits :: pct, false);
      } /*else if (Controller.ButtonL1.pressing()){
          Arm.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      } else if (Controller.ButtonL2.pressing()){
          Arm.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      }*/
}

void runIntake() {
      if(Controller.ButtonR1.pressing()) {
            Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::rpm);
        }else if(Controller.ButtonR2.pressing()) {
            Intake.spin(vex::directionType::rev,100,vex::velocityUnits::rpm);
        } else (Intake.stop());
}

void setIntake(double power) {
    Intake.spin(vex::directionType::rev, power, vex::velocityUnits::rpm);
}

void setLeftDrive(double power) {
     FrontLeftDrive.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
     BackLeftDrive.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
}

void setRightDrive(double power) {
     FrontRightDrive.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
     BackRightDrive.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
    
}

void init() {
      FrontLeftDrive.stop(vex::brakeType::coast);
    BackLeftDrive.stop(vex::brakeType::coast);
    FrontRightDrive.stop(vex::brakeType::coast);
    BackRightDrive.stop(vex::brakeType::coast);
    
    ShooterRight.stop(vex::brakeType::coast);
    ShooterLeft.stop(vex::brakeType::coast);
    
    Intake.stop(vex::brakeType::coast);
    
    Arm.stop(vex::brakeType::brake);
}


