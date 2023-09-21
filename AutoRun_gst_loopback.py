#!/usr/bin/python3
import subprocess
from subprocess import Popen, PIPE
import time
import threading
import os
import requests

camDev = []
devID = []
camStatus = []
p = subprocess.Popen('lsusb', shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
output = p.communicate()[0]
usbDevices = output.decode('utf-8')
#print(usbDevices)
for usbDev in usbDevices.splitlines():
	if 'Ricoh' in usbDev:
		camDev.append(usbDev)
		devID.append(usbDev[15:18])
		camStatus.append(usbDev[28:32])
		print(usbDev[15:18])
		print(usbDev[28:32])

for cid in devID:
	print("----------" + cid + ": status-------")
	cmd = 'ptpcam --dev=' + cid + ' --show-property=0x5013'
	p = subprocess.Popen(cmd, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
	output = p.communicate()[0]
	status = output.decode('utf-8')
	captureMode=False
	for s in status.splitlines():
		print(s)
		if 'ERROR' in s:
			print('Wake up')
			cmd2 = 'ptpcam --dev=' + cid + ' --set-property=0xD80E --val=0x00'
			pw = subprocess.Popen(cmd2, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
			outp = pw.communicate()[0]
			print(outp.decode('utf-8'))

			print('set live mode')
			cmd3 = 'ptpcam --dev=' + cid + ' --set-property=0x5013 --val=0x8005'
			pl = subprocess.Popen(cmd3, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
			outp = pl.communicate()[0]
			print(outp.decode('utf-8'))

		if 'Capture' in s:
			captureMode=True

	if captureMode==False:
		print('set live mode')
		cmd3 = 'ptpcam --dev=' + cid + ' --set-property=0x5013 --val=0x8005'
		pl = subprocess.Popen(cmd3, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
		outp = pl.communicate()[0]
		print(outp.decode('utf-8'))

print("Num of Cam:", len(devID))

cam0 = '/home/tristar/MyWork-NX4_6/THETA_Cameras/camera0/gst_loopback'
# cam0 = './gstrec/gst_loopback'
runTheta0 = True
def openTHETA0():
	print('Open THETA0')
	p0 = subprocess.Popen(cam0, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
	print('theta0:', p0.stdout.readline().decode().strip())
	while runTheta0:
		if p0.poll() is None:
			print('theta0:', p0.stdout.readline().decode().strip())

	p0.terminate()
	p0.wait()
	print("end camera0")

cam_rec = './gstrec/gst_viewer'
runThetaRec = True
def openTHETARec():
	print('Rec THETA0')
	p_rec = subprocess.Popen(cam_rec, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
	print('theta0:', p_rec.stdout.readline().decode().strip())
	while runTheta0:
		if p_rec.poll() is None:
			print('theta0:', p_rec.stdout.readline().decode().strip())

	p_rec.terminate()
	p_rec.wait()
	print("end rec canera")


if __name__ == '__main__':
	theta0_Th = threading.Thread(target = openTHETA0)
	theta0_Th.start()
	# time.sleep(10)
	# thetaRec_Th = threading.Thread(target = openTHETARec)
	# thetaRec_Th.start()
	# time.sleep(3)
	mainRun = True
	try:
		while mainRun:
			time.sleep(10)
	except KeyboardInterrupt:
		runTheta0 = False
		time.sleep(2)
		mainRun = False
		print("------END--------Press Ctrl+C again if not end")
