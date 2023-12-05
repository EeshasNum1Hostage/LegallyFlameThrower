package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class CollecterMovement {
    private final CANCoder absoluteEncoder;
    private final WPI_TalonFX collect;


    public void move(double p){
        collect.set(p);
    }

    public CollecterMovement(int id, int cid){
        collect = new WPI_TalonFX(id);
        absoluteEncoder = new CANCoder(cid);
    }

    private boolean UpLimitHit(){
        return Constants.UPCOLLECTERLIMIT >= (Math.toRadians(absoluteEncoder.getAbsolutePosition()) + Constants.COLLECTINITIALDEGREE);
    }

    private boolean DownLimitHit(){
        return Constants.LOWCOLLECTERLIMIT <= (Math.toRadians(absoluteEncoder.getAbsolutePosition()) + Constants.COLLECTINITIALDEGREE);
    }

    public void goUp(){
        while(UpLimitHit()) {
            collect.set(5);
        }
        collect.set(0);
        return;
    }

    public void goDown(){
        while(DownLimitHit()) {
            collect.set(-5);
        }
        collect.set(0);
        return;
    }

}
