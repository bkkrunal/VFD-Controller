# VFD-Controller
This project is developed to control the speed of the three phase water pumping motor using VFD with respect to pressure

We have two solar parabolic reflectors installed at our premises.
We are using them for heating the water as well as for cooking for a group of 150-200 people.

The solar reflectors are concentrating their energy in the form of heat on the metal receiver which gets heated up to 250° - 300° C.
Then the water is pumped through these receivers using piston pump and motor. Due to the heat of the receiver, water gets heated and is converted into steam.

This steam is then taken to the steam accumulator, where we are storing the energy in presurrised accumulator so that it can be used for the whole day.
The input pressure to this accumulator needs to be fixed at 5 BAR. 
This input pressure to the accumulator is dependent on the frequncy of the motor as well as the usage of the steam from the accumulator. 
When the usage is more, the pressure drps and when the usage is less the pressure rises.

So to keep this pressure fixed at 5 BAR, a system is designed using Arduino and Ethernet shield, Let us call it Arduino VFD Controller.

The system needs to monitor the pressure to the accumulatr and accordingly vary the frequncy of the motor using VFD.
A Graphical User Interface(GUI) is also provided to the user to control the Motors as well as to monitor the frequency the VFD.

Two VFDs are needed for two dishes and a pressure sensor at the input of the accumulator.
The Arduino VFD Controller is kept with VFD & three phase motor which are near dishes.
The actual distance between the pressure accumulator and these solar dishes is too much and thus pressure sensor can not be connected directly to the Arduino.
So one system was designed just to read the pressure and send it to the Arduino VFD Controller.
The communication between these two systems as well as communication to the GUI is implemented on ethernet layer using UDP protocol.
Arduino Ethernet shiled with W5100 is used for this purpose.

To implement pressure regultor with arduino PID logic is implemented using PID library from...
http://playground.arduino.cc/Code/PIDLibrary

One Arduino is going to control two VFDs. So two PID instances were created.
According to the input pressure, the PID logic will calculate the output frequncy for both the VFDs. 

To control the frequency of the motor, the analog port of the VFDs could have been used.
But it requires 0-10V signal level,which would have added another voltage regulator & a DAC with Arduino.
So RS485 communication link is used to communicate between VFD and Arduino using MODBUS protocol.
VFD is from FUJI drives with inbuit MODBUS protocol over RS485 communication link.

FUJI MAKE VFD part Number:
FRN0007C2S-4A

Now the actual flow goes like:

1. Arduino VFD Controller will get pressure on UDP port over ethernet from another pressure sender installed at accumulator...

2. It will process using PID logic and will give two output frequency for two VFDs...

3. These two output frequency will be then sent to the two VFDs using MODBUS protocol on RS485 link...

4. VFDs will be driving at these new frequency...

5. Meanwhile if user wants to monitor or control the VFDs run using GUI: GUI will be communicating with the Arduino VFD Controller using UDP protocol. Arduino will be sending this information to the GUI as well as will stop or run the motor as per           commanded from the GUI...

Three GUIs have been developed to test the code:
The GUIs can be found here...
https://github.com/bkkrunal/TestingGUIs

Two GUIs have been developed just to simulate Pressure sender and VFD to test the development...

1. Pressure Sender Simulator: It is acting as Pressure sender installed near accumulator. This GUI only send pressure to the Arduino VFD Controller. Pressure can be varied with track bar from 0 - 8 BAR in the step of 0.01 BAR. This GUI works on UDP protocol...

2. MODBUS Simulator: It is acting like VFD. Only Write one register of MODBUS fuction can be tested with this GUI. This GUI works on MODBUS protocol. This should be connected to the serial port of the Arduino VFD Controller while any other program using serial port of the Arduino should be closed before...


3. VFD Controller GUI shows the frequency of the both the motors, Also it shows the connection status of the VFDs. If any VFD is disconnected, it will show that in Red color else will be showing green in normal operation.
User can even control each motor and can run/stop individual motor.
We need to specify the IP address of the Arduino VFD controller to this GUI using settings tab(Default IP: 192.168.23.200).
This GUI is not a fuctional requirement for the Arduino VFD Controller, but it is just for monitoring purpose.

To simulate the system without VFD:
To simulate the system you will need two computers with different IP address. One to act as server for monitoring & other to act as pressure sender.

1. Hookup the Ethernet shield to the Arduino. Don't forget to short the INT jumper on bottom side of the official Arduino ethernet shield, as we are using interrupt based communication.

2. Connect Arduino to the USB port. Add ArduinoPID library in library folder of Arduino, compile & then upload the sketch. Close Arduino IDE. Connect ethernet wire to the ethernet shield. It should get in the network. Try to ping 192.168.23.200...

3. Open MODBUS Simulator GUI & connect it to the serial port of the Arduino. 

4. Open Pressure sender GUI in the same computer(This computer IP should be: 192.168.23.210).Run the GUI and Enter IP address as 192.168.23.200 & port as 1937. Press "Start Sending" button and it will start sending the pressure to 192.168.23.200 that is Arduino VFD controller.

5. Open VFD Controller GUI in another computer(This computer IP should be: 192.168.23.211). Change the IP addres in settings tab to 192.168.23.200 & Port to 1937. Restart the GUI. Now, you can see the frequncy of both motors.

6. Now if you close  MODBUS Simulator GUI, the background of the VFD dials in the VFD Controller GUI will turn red indicating that the communication link is broken. If you again start the  MODBUS Simulator GUI, the background color of the dials will turn green.




Testing the system with VFD:

1. Hookup the Ethernet shield to the Arduino. Don't forget to short the INT jumper on bottom side of the official Arduino ethernet shield, as we are using interrupt based communication.

2. Connect Arduino to the USB port. Add ArduinoPID library in library folder of Arduino, compile & then upload the sketch. Close Arduino IDE. Connect ethernet wire to the ethernet shield. It should get in the network. Try to ping 192.168.23.200...
 
3. For RS485 link,any RS485 shield can be used. Code uses standard hardware arduino serial pins (0 & 1) for communicating over RS485 with pin 8 as direction control of RS485 as it is half duplex system.

4. Power up VFD using 3phase supply & connect its RS485 output to the arduino RS485 output (A+ goes to A+ & B- goes to B-).
(BE CAREFUL WITH THE THREE PHASE SUPPLY. CONNECT UNDER SUPERVSION OF EXPERIENCED ELECTRICIAN)

5. Also power up the pressure sender and have it in the network. (I have implemented pressure sender using another arduino+ ethernet shield + analog pressure sensor)

6. If everything is OK, it will start rotating the motor according to the pressure input. 


Default IPs:
Arduino VFD Controller: 192.168.23.200... 
Pressure Sender: 192.168.23.210... 
Server IP(VFD Controller GUI): 192.168.23.211... 
Default UDP Port: 1937... 
