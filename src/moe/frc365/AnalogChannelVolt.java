/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package moe.frc365;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author dquam
 */
public class AnalogChannelVolt extends AnalogChannel {

    private static final double rev = 5.0;
    private static final double halfrev = rev / 2.;
    private static final double scale = rev / (4.8 - 0.2);
    private static final int ratio = 2; // ratio of pot to finished gear, must be int
    private AnalogTrigger m_trig;
    private AnalogTrigger m_trig2;
    private AnalogTriggerOutput m_trigo;
    private AnalogTriggerOutput m_trig2o;
    private Counter m_count;

    public AnalogChannelVolt(int moduleNumber, int channel) {
        super(moduleNumber, channel);

        this.getModule().setSampleRate(1000);

        m_trig = new AnalogTrigger(moduleNumber, channel);
        m_trig.setFiltered(true);
        m_trig.setLimitsVoltage(1.35, 3.65);

        m_count = new Counter();
        m_count.setUpDownCounterMode();
        m_count.setUpSource(m_trig, AnalogTriggerOutput.Type.kFallingPulse);
        m_count.setDownSource(m_trig, AnalogTriggerOutput.Type.kRisingPulse);
        m_count.start();
    }

    public double getAverageVoltage() {
        return getVoltage();
    }

    public void resetTurns() {
        m_count.reset();
        m_count.start();
    }

    public void start() {
    }

    public double getVoltage() {
        double temp = super.getVoltage();
        temp = (((temp - halfrev) * scale) + halfrev);  // scale
        if (temp < 0) {
            temp = 0; // min
        } else if (temp > rev) {
            temp = rev; // max
        }
        temp = (temp / ratio) + (Math.abs(m_count.get() % ratio) * halfrev); // half scale
        temp = rev - temp; // inverse
        return temp;
    }
    
    public int getTurns() {
        return m_count.get();
    }
    
    public double pidGet() {
        return getVoltage();
    }
}
