package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpinDex {
    private final CANCoder absoluteEncoder;
    private final WPI_TalonFX spinyThing;

    public SpinDex(int cid, int mid){
        absoluteEncoder = new CANCoder(cid);
        spinyThing = new WPI_TalonFX(mid);
    }

    public void spin(double p){
        spinyThing.set(p);
    }

}
