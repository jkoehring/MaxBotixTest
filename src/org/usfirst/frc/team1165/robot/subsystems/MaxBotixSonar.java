package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.commands.Reporter;
import org.usfirst.frc.team1165.util.SampleRate;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for obtaining data from a MaxBotix MB1013 Range Finder
 */
public class MaxBotixSonar extends ReportableSubsystem implements Runnable
{
	public enum Model
	{
		MB1013,
		MB1200
	}
	
	private Model model;
	
	private SerialPort serialPort;
	private double serialRange = -1;
	private double serialRangeFactor;
	private SampleRate serialSampleRate;
	
	private AnalogInput analogInput;
	private double analogRangeFactor;
	
	/**
	 * Constructs an instance using a serial port and/or analog input to
	 * get data from the sonar.
	 */
	public MaxBotixSonar(Model model, SerialPort serialPort, AnalogInput analogInput)
	{
		this.model = model;
		this.serialPort = serialPort;
		this.analogInput = analogInput;
		if (null != serialPort)
		{
			serialSampleRate = new SampleRate();
			serialSampleRate.start();
			serialRangeFactor = (Model.MB1013 == model)
				? 25.4		// convert from mm to inches
				: 2.54;		// convert from cm to inches
			new Thread(this).start();
		}
		
		if (null != analogInput)
		{
			analogRangeFactor = (Model.MB1013 == model)
				? 25.4 / 1.25	// 25.40mm/inch / 1.25mm/bit  =  bits/inch
				: 2.54 / .25;	//  2.54cm/inch / 0.25cm/bit  =  bits/inch
		}
	}

	/**
	 * Reads and returns the current range (in inches) from the analog port.
	 * -1 is returned if range is not being obtained form the analog port.
	 */
	public double getAnalogRange()
	{
		return analogInput == null ? -1 : analogInput.getValue() / analogRangeFactor;
	}

	/**
	 * Reads and returns the current range (in inches) from the serial port.
	 * -1 is returned if range is not being obtained form the serial port.
	 */
	public double getSerialRange()
	{
		return serialRange;
	}
	
	public void initDefaultCommand()
	{
		// Set the default command for a subsystem here.
		setDefaultCommand(new Reporter(this));
	}

	@Override
	public void report()
	{
		if (null != serialPort)
		{
			SmartDashboard.putNumber(model + " Serial", getSerialRange());
			SmartDashboard.putNumber(model + " Rate", serialSampleRate.getSampleRate());
		}
		
		if (null != analogInput)
		{
			SmartDashboard.putNumber(model + " Analog", getAnalogRange());
		}

	}

	/**
	 * Thread that reads from the serial port.
	 */
	@Override
	public void run()
	{
		// Reset serial port to empty buffers:
		serialPort.reset();
		
		int value = 0;
		byte[] data = new byte[0];
		int index = 0;
		boolean startByteFound = false;
		
		while (true)
		{
			if (index >= data.length)
			{
				data = serialPort.read(serialPort.getBytesReceived());
				if (0 == data.length)
				{
					continue;
				}
				index = 0;
			}
			
			byte c = data[index++];
			if (startByteFound)
			{
				if ('\r' == c)
				{
					serialRange = value / serialRangeFactor;
					value = 0;
					startByteFound = false;
					serialSampleRate.addSample();
				}
				else
				{
					value = value * 10 + (c - '0');
				}
			}
			else
			{
				startByteFound = 'R' == c;
			}
		}
	}
}
