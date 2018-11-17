#include "robot-config.h"
#include "my_motor.cpp"

MyMotor t(vex::PORT7, vex::gearSetting::ratio18_1, false);
          
vex::competition Competiton;

PIDControllerWithFeedForward pid(0.3, 0.0, 0.0, 1.0);

void pre_auton (void){
}

void shoot(double goal) {
   double shooterSpeed = (ShooterRight.velocity(vex::velocityUnits::rpm) + ShooterLeft.velocity(vex::velocityUnits::rpm)) / 2.0;
   while (abs(shooterSpeed - goal) < 5) {
       shooterSpeed = (ShooterRight.velocity(vex::velocityUnits::rpm) + ShooterLeft.velocity(vex::velocityUnits::rpm)) / 2.0;
       double power = pid.update(shooterSpeed, goal);
       ShooterRight.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
       ShooterLeft.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
   }
   
}


double inchesToEncoders(double encoders) {
    return encoders * 0.88 / (PI * 4);
}

void driveRight(double distance, double speed) {
    FrontRightDrive.rotateFor(distance, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm, false);
    BackRightDrive.rotateFor(distance, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
}

void driveLeft(double distance, double speed) {
    FrontLeftDrive.rotateFor(distance, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm, false);
    BackLeftDrive.rotateFor(distance, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm, false);    
}

void driveForward(double distance, double speed) {
    distance = inchesToEncoders(distance);
   
    driveLeft(distance, speed);
    driveRight(distance, speed);
}

double turnSpeed = 100;
double turn90 = 0.93309;
double turn45 = 0.673309;
double turn135 = turn90 + turn45;

void turn(double angle, bool right, double turnSpeed) {
  if (right) {
      driveLeft(angle, turnSpeed);
      driveRight(-angle, turnSpeed);
  } else {
      driveLeft(-angle, turnSpeed);
      driveRight(angle, turnSpeed);
  }
}

bool red = false;

void lowFlagAndClimb() {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Auto");
    driveForward(49,  200);
    Controller.Screen.print("Went forward");
    driveForward(-85, 200);
    Controller.Screen.print("Went back");
    if (red) {
        turn(turn90, true, 150);
    } else {
        turn(turn90, false, 150);
    }
    Controller.Screen.print("Turned");
    driveForward(60,200);
    Controller.Screen.print("Forward again");
}

void highFlagAndClimb() {
     Brain.Screen.clearScreen();
    Brain.Screen.print("Auto");
    driveForward(50,  200);
    Controller.Screen.print("Went forward");
    driveForward(-30, 200);
    Controller.Screen.print("Went back");    
}

void backSide() {
    driveForward(45, 150);
    Intake.rotateFor(-1, vex::rotationUnits::rev, 200, vex::velocityUnits::rpm, false);
    vex::task::sleep(4.0);
    driveForward(-12, 100);
    Intake.rotateFor(1, vex::rotationUnits::rev, 200, vex::velocityUnits::rpm, false);
    if (red) {
        turn(turn45, true, 150);
    } else {
        turn(turn45, false, 150);
    }
   Intake.rotateFor(3, vex::rotationUnits::rev, 200, vex::velocityUnits::rpm, false);
   driveForward(30, 100);
   vex::task::sleep(4);
   driveForward(-30, 100);
    if (red) {  
        turn(turn135, false, 150);
    } else {
        turn(turn135, true, 150);
    }
    driveForward(50, 200);
}

void autonomous(void) {  
  // lowFlagAndClimb();
   backSide();
}

void usercontrol (void){
    init();
    
    while (true){
       
      /* double shooterSpeed = (ShooterRight.velocity(vex::velocityUnits::rpm) + ShooterLeft.velocity(vex::velocityUnits::rpm)) / 2.0;
       double goal = -100;
       if (Controller.ButtonL1.pressing()) {
           goal = 60;
       } else if (Controller.ButtonL2.pressing()) {
           goal = 50;
           pid.setConstants(0.3, 0.0, 0.0, 1.0);
       } else if (Controller.ButtonDown.pressing()) {
           goal = 35;
           pid.setConstants(0.2, 0.0, 0.0, 1.0);
       } else if (Controller.ButtonLeft.pressing()) {
           goal = -30;
       } 
        
       if (abs(pid.getError()) < 10 && goal > -90) {
           Controller.rumble("-");
       }
           
       if (goal > -90) {
           double power = pid.update(shooterSpeed, goal);
           ShooterRight.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
           ShooterLeft.spin(vex::directionType::fwd, power, vex::velocityUnits::pct);
       } else {
           ShooterRight.stop(vex::brakeType::coast);
           ShooterLeft.stop(vex::brakeType::coast);
       }
                
       Brain.Screen.print("Speed %f", shooterSpeed);
       Brain.Screen.newLine(); */
  
       
    //   runIntake();
    //   runArm();
    //   runDrive();
        if (Controller.ButtonL1.pressing()) {
            t.setPct(100);
        } else if (Controller.ButtonL2.pressing()) {
            t.setPct(-100);
        } else {
            t.setPct(0);
        }
       Brain.Screen.print(t.getPosition());
       Brain.Screen.newLine();
       vex::task::sleep(20);
        
    }
}

int main(){
    
    pre_auton();
    Competiton.autonomous( autonomous);
    Competiton.drivercontrol( usercontrol );  
    while (1){
        vex::task::sleep(100);
    }
}




