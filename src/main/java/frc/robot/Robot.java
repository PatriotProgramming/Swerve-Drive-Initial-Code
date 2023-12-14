package frc.robot;
import java.io.IOException;
import java.nio.file.Path;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Wheel 1 is front Left, 2 front Right, 3 back Right, 4 back Left


public class Robot extends TimedRobot {

  private double V1, V2, V3, V4, Vm;
  //V final speed, V1 speed wheel 1, Vm = max velocity

  // private double V1x, V1y, V2x, V2y, V3x, V3y, V4x, V4y;

  private double L, W, r, w, T;
  //Length and Width of robot, unit of measurement doesn't matter, R is hypotenuse of tan(L/W)
  //w rotation speed radians/s

  private double A, B, C, D;
  //4 equations needed

  private XboxController xbox;

  private double FWD, STR, RCW, forward, straferight;
  //FWD forward, STR strafe right, RCW Rotate clockwise

  private double W1a, W2a, W3a, W4a;
  //Needed Wheel 1 angle, ect

  private CANSparkMax FLS, FRS, BLS, BRS, FLA, FRA, BLA, BRA;
  //each motor, Front Left Speed motor, Front Left Angle motor

  private CANCoder FLAE, FRAE, BLAE, BRAE;
  //encodors for each motor


  private PIDController pid;

  private AHRS navx;

  private double temp, currentAngle;

  private double DFL, DFR, DBL, DBR;

  private double frontRightAngleEncoderDeg, frontLeftAngleEncoderDeg, backRightAngleEncoderDeg, backLeftAngleEncoderDeg;


  public NetworkTable table;
  public NetworkTableEntry tx, ty, ta, tv;

  public double distance, cameraAngle = 31, cameraHeight = 5.5, targetHeight = 24;

  @Override
  public void robotInit() 
  {
    xbox = new XboxController(0);

    FRS = new CANSparkMax(3, MotorType.kBrushless);
    FRS.setInverted(true);
    FRS.setSmartCurrentLimit(30);

    FLS = new CANSparkMax(2, MotorType.kBrushless);
    FLS.setInverted(false);
    FLS.setSmartCurrentLimit(30);

    BLS = new CANSparkMax(8, MotorType.kBrushless);
    BLS.setInverted(false);   
    BLS.setSmartCurrentLimit(30);

    BRS = new CANSparkMax(6, MotorType.kBrushless);
    BRS.setInverted(true);
    BRS.setSmartCurrentLimit(30);

    FRA = new CANSparkMax(4, MotorType.kBrushless);
    FRA.setIdleMode(IdleMode.kCoast);
    // FRA.setSmartCurrentLimit(30);
    FLA = new CANSparkMax(1, MotorType.kBrushless);
    FLA.setIdleMode(IdleMode.kCoast);  
    // FLA.setSmartCurrentLimit(30);
    BLA = new CANSparkMax(7, MotorType.kBrushless);
    BLA.setIdleMode(IdleMode.kCoast);
    // BLA.setSmartCurrentLimit(30);
    BRA = new CANSparkMax(5, MotorType.kBrushless);
    BRA.setIdleMode(IdleMode.kCoast);
    // BRA.setSmartCurrentLimit(30);

    FRAE = new CANCoder(1);
    FLAE = new CANCoder(2);
    BLAE = new CANCoder(4);
    BRAE = new CANCoder(3);

    

    L = 19.5;
    W = 19.5;

    r = Math.sqrt((L*L) + (W*W));
    w = 3000;

    T = r * w;

    pid = new PIDController(.003, 0, 0);//.005,0,0 //.009, .045, .00045
    pid.enableContinuousInput(-180, 180);

    navx = new AHRS();

    navx.zeroYaw();

    FRA.getEncoder().setPosition(0);
    FLA.getEncoder().setPosition(0);
    BRA.getEncoder().setPosition(0);
    BLA.getEncoder().setPosition(0);

    table = NetworkTableInstance.getDefault().getTable("limelight-daza");
    
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    //Arm

  }


  @Override
  public void robotPeriodic() 
  {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    double angle = cameraAngle + y;
    distance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angle));
    
    DFR = FRAE.getAbsolutePosition();
    DFL = FLAE.getAbsolutePosition();
    DBL = BLAE.getAbsolutePosition();
    DBR = BRAE.getAbsolutePosition();

    //Button Bindings
    forward = -xbox.getLeftY();
    straferight = -xbox.getLeftX();
    if(xbox.getRightX() > 0.05)
    {
      RCW = -xbox.getRightX();
    }
    else if(xbox.getRightX() < -0.05)
    {
      RCW = -xbox.getRightX();
    }
    else
    {
      RCW = 0;
    }

    double gyro_degrees = -navx.getYaw();
    double gyro_radians = gyro_degrees * (Math.PI / 180); 
    double temp = forward * Math.cos(gyro_radians) + straferight * Math.sin(gyro_radians);
    STR = -forward * Math.sin(gyro_radians) + straferight * Math.cos(gyro_radians);
    FWD = temp;

    A = STR - RCW * (L/r);
    B = STR + RCW * (L/r);
    C = FWD - RCW * (W/r);
    D = FWD + RCW * (W/r);

    // V1x = B;
    // V1y = C;
    V1 = Math.sqrt(B*B + C*C); //calculates final velocity of motor
    W1a = Math.atan2(B,C) * 180/(Math.PI); //calculates angle of motor needed for resultant robot velocity and angle

    // V2x = B;
    // V2y = D;
    V2 = Math.sqrt(B*B + D*D);
    W2a = Math.atan2(B,D) * 180/(Math.PI);

    // V3x = A;
    // V3y = D;
    V3 = Math.sqrt(A*A + D*D);
    W3a = Math.atan2(A,D) * 180/(Math.PI);
    

    // V4x = A;
    // V4y = C;
    V4 = Math.sqrt(A*A + C*C);
    W4a = Math.atan2(A,C) * 180/(Math.PI);

    
    //normalizes velocities to be relative to eachother when Vm > 1
    Vm = V1;
    if(V2 > Vm)
    {
      Vm=V2;
    } 
    if(V3>Vm)
    {
      Vm=V3;
    } 
    if(V4>Vm)
    {
      Vm=V4;
    }
    if(Vm>1)
    {
      V1 /= Vm; //divide V1 by Vm
      V2 /= Vm; 
      V3 /= Vm; 
      V4 /= Vm;
    }



    if(xbox.getRightTriggerAxis() > 0.5)
    {

      FRA.set(pid.calculate(DFR, W2a));
      FLA.set(pid.calculate(DFL, W1a));
      BLA.set(pid.calculate(DBL, W4a)); //pid.calculate(DBL, W3a)
      BRA.set(pid.calculate(DBR, W3a));

      FRS.set(V2 * .65);
      FLS.set(V1 * .65);
      BLS.set(V4 * .65);
      BRS.set(V3 * .65);

      
    }
    else
    {
      FRS.stopMotor();
      FLS.stopMotor();
      BLS.stopMotor();
      BRS.stopMotor();
      FRA.stopMotor();
      FLA.stopMotor();
      BLA.stopMotor();
      BRA.stopMotor();
    }

    if(xbox.getRawButton(1))
    {
      BLS.getEncoder().setPosition(0);
      BRS.getEncoder().setPosition(0);
      FLS.getEncoder().setPosition(0);
      FRS.getEncoder().setPosition(0);
    }

    frontRightAngleEncoderDeg = (((FRA.getEncoder().getPosition() % 1) + 1) % 1) * 360;
    frontLeftAngleEncoderDeg = (((FLA.getEncoder().getPosition() % 1) + 1) % 1) * 360;
    backRightAngleEncoderDeg = (((BRA.getEncoder().getPosition() % 1) + 1) % 1) * 360;
    backLeftAngleEncoderDeg = (((BLA.getEncoder().getPosition() % 1) + 1) % 1) * 360;


    

    SmartDashboard.putNumber("Front Right Angle CANcoder", FRAE.getAbsolutePosition());
    SmartDashboard.putNumber("Front Left Angle CANcoder", FLAE.getAbsolutePosition());
    SmartDashboard.putNumber("Back Left Angle CANcoder", BLAE.getAbsolutePosition());
    SmartDashboard.putNumber("Back Right Angle CANcoder", BRAE.getAbsolutePosition());

    SmartDashboard.putNumber("Front Right Angle motor encoder", frontRightAngleEncoderDeg);
    SmartDashboard.putNumber("Front Left Angle motor encoder", frontLeftAngleEncoderDeg);
    SmartDashboard.putNumber("Back Left Angle motor encoder", backRightAngleEncoderDeg);
    SmartDashboard.putNumber("Back Right Angle motor encoder", backLeftAngleEncoderDeg);

    SmartDashboard.putNumber("A", A);
    SmartDashboard.putNumber("B", B);
    SmartDashboard.putNumber("C", C);
    SmartDashboard.putNumber("D", D);

    SmartDashboard.putNumber("Wanted 1 angle", W1a);
    SmartDashboard.putNumber("Wanted 2 angle", W2a);
    SmartDashboard.putNumber("Wanted 3 angle", W3a);
    SmartDashboard.putNumber("Wanted 4 angle", W4a);

    SmartDashboard.putNumber("V1", V1);
    SmartDashboard.putNumber("V2", V2);
    SmartDashboard.putNumber("V3", V3);
    SmartDashboard.putNumber("V4", V3);
    
    SmartDashboard.putNumber("Front Left Angle Power", pid.calculate(DFR, W1a));
    SmartDashboard.putNumber("Front Right Angle Power", pid.calculate(DFL, W2a));
    SmartDashboard.putNumber("Back Left Angle Power", pid.calculate(DBL, W3a));
    SmartDashboard.putNumber("Back Right Angle Power", pid.calculate(DBR, W4a));
    
    SmartDashboard.putNumber("FWD", FWD);
    SmartDashboard.putNumber("STR", STR);
    SmartDashboard.putNumber("RCW", RCW);

    SmartDashboard.putNumber("NAVX Angle", navx.getYaw());

    SmartDashboard.putNumber("Front Left Speed Encoder", FLS.getEncoder().getPosition());
    SmartDashboard.putNumber("Front Right Speed Encoder", FRS.getEncoder().getPosition());
    SmartDashboard.putNumber("Back Left Speed Encoder", BLS.getEncoder().getPosition());
    SmartDashboard.putNumber("Back Right Speed Encoder", BRS.getEncoder().getPosition());

    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Horizontal Offset", x);
    SmartDashboard.putNumber("VertOff", y);
    SmartDashboard.putNumber("Area", area);
    

  }
}
