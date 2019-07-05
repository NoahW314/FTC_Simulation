package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantReadWriteLock.ReadLock;

import org.firstinspires.ftc.robotcore.internal.hardware.TimeWindow;
import org.firstinspires.ftc.robotcore.internal.usb.FakeSerialNumber;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule;
import com.qualcomm.robotcore.util.SerialNumber;

public class I2cController implements com.qualcomm.robotcore.hardware.I2cController, RobotUsbModule{

	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}

	@Override
	public String getDeviceName() {
		return "I2c Controller Simulation";
	}

	@Override
	public String getConnectionInfo() {
		return "Connection Info doesn't exist for a Simulated I2c Controller";
	}

	@Override
	public int getVersion() {
		return 0;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
	}

	@Override
	public void close() {
	}

	@Override
	public SerialNumber getSerialNumber() {
		return new FakeSerialNumber();
	}

	@Override
	public void enableI2cReadMode(int physicalPort, I2cAddr i2cAddress, int memAddress, int length) {
	}

	@Override
	public void enableI2cWriteMode(int physicalPort, I2cAddr i2cAddress, int memAddress, int length) {
	}

	@Override
	public byte[] getCopyOfReadBuffer(int physicalPort) {
		return new byte[10];
	}

	@Override
	public byte[] getCopyOfWriteBuffer(int physicalPort) {
		return new byte[10];
	}

	@Override
	public void copyBufferIntoWriteBuffer(int physicalPort, byte[] buffer) {
	}

	@Override
	public void setI2cPortActionFlag(int port) {
	}

	@Override
	public void clearI2cPortActionFlag(int port) {
	}

	@Override
	public boolean isI2cPortActionFlagSet(int port) {
		return false;
	}

	@Override
	public void readI2cCacheFromController(int port) {
	}

	@Override
	public void writeI2cCacheToController(int port) {
	}

	@Override
	public void writeI2cPortFlagOnlyToController(int port) {
	}

	@Override
	public boolean isI2cPortInReadMode(int port) {
		return false;
	}

	@Override
	public boolean isI2cPortInWriteMode(int port) {
		return false;
	}

	@Override
	public boolean isI2cPortReady(int port) {
		return false;
	}

	@Override
	public Lock getI2cReadCacheLock(int port) {
		return null;
	}

	@Override
	public Lock getI2cWriteCacheLock(int port) {
		return null;
	}

	@Override
	public byte[] getI2cReadCache(int port) {
		return new byte[10];
	}

	@Override
	public TimeWindow getI2cReadCacheTimeWindow(int port) {
		return new TimeWindow();
	}

	@Override
	public byte[] getI2cWriteCache(int port) {
		return new byte[10];
	}

	@Override
	public int getMaxI2cWriteLatency(int port) {
		return 0;
	}

	@Override
	public void registerForI2cPortReadyCallback(I2cPortReadyCallback callback, int port) {
	}

	@Override
	public I2cPortReadyCallback getI2cPortReadyCallback(int port) {
		return new I2cPortReadyCallback() {
			@Override
			public void portIsReady(int port) {
			}
		};
	}

	@Override
	public void deregisterForPortReadyCallback(int port) {
	}

	@Override
	public void registerForPortReadyBeginEndCallback(I2cPortReadyBeginEndNotifications callback, int port) {
	}

	@Override
	public I2cPortReadyBeginEndNotifications getPortReadyBeginEndCallback(int port) {
		return null;
	}

	@Override
	public void deregisterForPortReadyBeginEndCallback(int port) {
	}

	@Override
	public boolean isArmed() {
		return false;
	}

	@Override
	public void readI2cCacheFromModule(int port) {
	}

	@Override
	public void writeI2cCacheToModule(int port) {
	}

	@Override
	public void writeI2cPortFlagOnlyToModule(int port) {
	}

	@Override
	public ARMINGSTATE getArmingState() {
		return ARMINGSTATE.PRETENDING;
	}

	@Override
	public void registerCallback(Callback callback, boolean doInitialCallback) {
	}

	@Override
	public void unregisterCallback(Callback callback) {
	}

	@Override
	public void arm() throws RobotCoreException, InterruptedException {
	}

	@Override
	public void pretend() throws RobotCoreException, InterruptedException {
	}

	@Override
	public void armOrPretend() throws RobotCoreException, InterruptedException {
	}

	@Override
	public void disarm() throws RobotCoreException, InterruptedException {
	}

}
