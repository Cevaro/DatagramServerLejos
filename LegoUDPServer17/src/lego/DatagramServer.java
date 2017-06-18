package lego;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.utility.Delay;

public class DatagramServer {

	private static final float MAX_DISTANCE = 0.4f;
	private static final int DETECTOR_DELAY = 20;

	public static RegulatedMotor mleft;
	public static RegulatedMotor mright;
	public static RegulatedMotor msensor;
	public static RegulatedMotor[] marray;
	public static Wheel wheel1;
	public static Wheel wheel2;
	public static Chassis chassis;
	public static MovePilot pilot;
	public static EV3IRSensor ir;
	public static EV3UltrasonicSensor us;
	public static SampleProvider irDistance;
	public static SampleProvider usDistance;
	public static float[] irSample;
	public static float[] usSample;
	public static boolean priorityModeFlag = false;
	public static float previousSample;
	public static int rotationTime;

	public static String s = null;
	public static float maxSpeedA = 0;
	public static float maxSpeedD = 0;

	public static void main(String[] args) throws IOException {
		mleft = new EV3LargeRegulatedMotor(MotorPort.A);
		mright = new EV3LargeRegulatedMotor(MotorPort.D);
		msensor = new EV3MediumRegulatedMotor(MotorPort.B);
		marray = new RegulatedMotor[1];
		marray[0] = mright;
		mleft.synchronizeWith(marray);
		mleft.setSpeed(10000);
		mright.setSpeed(10000);

		DatagramSocket socket = new DatagramSocket(4711);
		socket.setSoTimeout(5000);
		socket.setReceiveBufferSize(1);

		ir = new EV3IRSensor(SensorPort.S1);
		us = new EV3UltrasonicSensor(SensorPort.S4);
		irDistance = ir.getDistanceMode();
		usDistance = us.getDistanceMode();
		irSample = new float[irDistance.sampleSize()];
		usSample = new float[usDistance.sampleSize() * 9];
		SensorTurner turner = new SensorTurner();
		turner.start();

		irDistance.fetchSample(irSample, 0);
		LCD.drawString(Float.toString(irSample[0]), 0, 7);
		previousSample = irSample[0];

	
		System.out.println("Server ready...");
		while (true) {
			// Auf Anfrage warten

			priorityModeFlag = false;
			DatagramPacket packet = new DatagramPacket(new byte[16], 9);
			try {
				socket.receive(packet);

				// Empfänger auslesen

				// InetAddress address = packet.getAddress();
				// int port = packet.getPort();
				// int len = packet.getLength();
				byte[] data = packet.getData();

				s = new String(data);
				LCD.drawString(s, 0, 1);
				// Delay.msDelay(100);

				checkIRDistance();

				if(!priorityModeFlag){
				checkUSDistance();
				}
				
				if (!priorityModeFlag) {
					react(data);
				}
			} catch (SocketTimeoutException e) {
				mleft.startSynchronization();
				mleft.stop();
				mright.stop();
				mleft.endSynchronization();
				continue;
			}

		}
	}

	public static void checkUSDistance() {
		boolean objectInFront = false;
		boolean objectInLeft = false;
		boolean objectInRight = false;
			LCD.drawString(Float.toString(usSample[0]), 0, 3);
			LCD.drawString(Float.toString(usSample[4]), 0, 4);
			LCD.drawString(Float.toString(usSample[8]), 0, 5);
			
			if (usSample[3] < 0.25 || usSample[4] < 0.27 || usSample[5] < 0.25){
				objectInFront = true;
				//avoidFrontObject();
				priorityModeFlag = true;
			}
			if (usSample[0] < 0.15 || usSample[1] < 0.16 || usSample[2] < 0.15){
				objectInLeft = true;
				//avoidFrontObject();
				priorityModeFlag = true;
			}
			if (usSample[6] < 0.15 || usSample[7] < 0.16 || usSample[8] < 0.15){
				objectInRight = true;
				//avoidFrontObject();
				priorityModeFlag = true;
			}
			
			if (objectInFront == false && objectInLeft == true && objectInRight == false){
				Button.LEDPattern(1);
				turnLeft(500);
			} else if (objectInFront == false && objectInLeft == false && objectInRight == true){
				Button.LEDPattern(1);
				turnRight(500);
			} else if (objectInFront == true && objectInLeft == true && objectInRight == false){
				Button.LEDPattern(1);
				turnLeft(1000);
			} else if (objectInFront == true && objectInLeft == false && objectInRight == true){
				Button.LEDPattern(1);
				turnRight(1000);
			} else if (objectInFront == true && objectInLeft == true && objectInRight == true){
				startBackwardManeuver(1);
			} else if (objectInFront == true && objectInLeft == false && objectInRight == false){
				avoidFrontObject();
			}
			
	}

	public static void checkIRDistance() {
		irDistance.fetchSample(irSample, 0);
		LCD.drawString(Float.toString(irSample[0]), 0, 7);

		if ((irSample[0] * 0.9) > previousSample || (irSample[0] * 1.1) < previousSample) {
			startBackwardManeuver(0);
			priorityModeFlag = true;
		} else {
			previousSample = irSample[0];
		}
	}
	
	public static void avoidFrontObject(){
		//check left side
		
			//decide for one side
		Button.LEDPattern(2);
			if ((usSample[0]+usSample[1]+usSample[2]+usSample[3]) < (usSample[5]+usSample[6]+usSample[7]+usSample[8])){
				turnLeft(1000);
			} else turnRight(1000);
	}

	private static void turnLeft(int d) {
		mleft.stop();
		mright.setSpeed((int) (mright.getMaxSpeed()/2));
		mright.forward();
		Delay.msDelay(d);
		Button.LEDPattern(0);
	}
	
	private static void turnRight(int d) {
mright.stop();
mleft.setSpeed((int) (mleft.getMaxSpeed()/2));
mleft.forward();
Delay.msDelay(d);
Button.LEDPattern(0);
	}


	public static void startBackwardManeuver(int c) {
		mleft.startSynchronization();
		mleft.stop();
		mright.stop();
		mleft.endSynchronization();
		if (c == 0){
		Sound.buzz();
		} else if (c ==1){
			Sound.twoBeeps();
		}
		int quarterSpeed = (int) (mleft.getMaxSpeed() / 4);
		mleft.setSpeed(quarterSpeed);
		mright.setSpeed(quarterSpeed);
		mleft.startSynchronization();
		mleft.backward();
		mright.backward();
		mleft.endSynchronization();
		Delay.msDelay(1000);
		mleft.startSynchronization();
		mleft.stop();
		mright.stop();
		mleft.endSynchronization();
		irDistance.fetchSample(irSample, 0);
		LCD.drawString(Float.toString(irSample[0]), 0, 7);
		previousSample = irSample[0];
		Sound.beep();
	}

	public static void react(byte[] command) {
		int intensity = 0;
		int direction = 0;

		if (command[0] == 70) { // F
			// pilot.forward();
			int speedRegulated = (int)(mleft.getMaxSpeed()*0.8);
			mleft.setSpeed(speedRegulated);		
			mright.setSpeed(speedRegulated);
			mleft.startSynchronization();
			mleft.forward();
			mright.forward();
			mleft.endSynchronization();
		} else if (command[0] == 76) { // L
			mright.setSpeed((int)(mright.getMaxSpeed()*0.8));
			mright.forward();
		} else if (command[0] == 82) { // R
			// pilot.arcForward(500);
			mleft.setSpeed((int)(mright.getMaxSpeed()*0.8));
			mleft.forward();
			// mleft.setSpeed(80);
			// mleft.backward();
			// mright.setSpeed(100);
			// mright.backward();
		} else if (command[0] == 66) { // B
			int speedRegulated = (int)Math.floor(mleft.getMaxSpeed()*0.8);
			mleft.setSpeed(speedRegulated);		
			mright.setSpeed(speedRegulated);
			mleft.startSynchronization();
			mleft.backward();
			mright.backward();
			mleft.endSynchronization();
			// pilot.backward();
			// mleft.setSpeed(100);
			// mleft.backward();
			//
			// mleft.setSpeed(100);
			// mright.backward();
		} else if (command[0] == 83) { // S
			// pilot.stop();
			mleft.startSynchronization();
			mleft.stop();
			mright.stop();
			mleft.endSynchronization();
		} else if (command[0] == 71) { // G
			if (command[2] != 48)
				intensity = Integer.parseInt(new String(command, 2, 3));
			else if (command[3] != 48)
				intensity = Integer.parseInt(new String(command, 3, 2));
			else
				intensity = Integer.parseInt(new String(command, 4, 1));

			if (command[6] != 48)
				direction = Integer.parseInt(new String(command, 6, 3));
			else if (command[7] != 48)
				direction = Integer.parseInt(new String(command, 7, 2));
			else
				direction = Integer.parseInt(new String(command, 8, 1));

			intensity = (int) (Math.floor((mleft.getMaxSpeed() * intensity) / 100));
			float f = intensity * (1 - ((float) direction / 100));

			if (direction != 0) {
				if (command[5] == 76) {
					mleft.setSpeed((int) (f));
					mright.setSpeed(intensity);
				} else {
					mright.setSpeed((int) (f));
					mleft.setSpeed(intensity);
				}
			}
			mleft.startSynchronization();
			if (command[1] == 70) {
				mleft.forward();
				mright.forward();
			} else {
				mleft.backward();
				mright.backward();
			}
			mleft.endSynchronization();

		}
	}

	public static class SensorTurner extends Thread{
		
		public void run() {
			//msensor.setSpeed((int) msensor.getMaxSpeed());
			msensor.rotateTo(75);
			 long startTime = System.currentTimeMillis();
			 msensor.rotateTo(-75);
			 long endTime = System.currentTimeMillis();
			 rotationTime = (int) (endTime - startTime);	
			while (true){
			msensor.rotateTo(75, true);
			for (int i = 0; i < usSample.length; i++) {
				usDistance.fetchSample(usSample, i);
				if (Float.isInfinite(usSample[i])) usSample[i] = 2;
				Delay.msDelay(rotationTime/9);
			}
			msensor.rotateTo(-75, true);
			for (int i = usSample.length - 1; i >= 0; i--) {
				usDistance.fetchSample(usSample, i);
				if (Float.isInfinite(usSample[i])) usSample[i] = 2;
				Delay.msDelay(rotationTime/9);			
				}
	    }
	}
}
}