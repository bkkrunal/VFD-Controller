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

So to keep this pressure fixed at 5 BAR, I have designed a system using Arduino and Ethernet shield.

The system needs to monitor the pressure to the accumulatr and accordingly vary the frequncy of the motor using VFD.
Graphical User Interface(GUI) is also provided for thr user to control the Motors as well as to monitor the frequency of two VFDs

Two VFDs are needed for two dishes and a pressure sensor at the input of the accumulator.
The actual distance between the accumulator and these solar dishes was too much and thus I could not connect the pressure sensor directly to the Arduino.
So one system was designed just to read the pressure and send it to the Arduino VFD Controller.
The communication between these two systems as well as communication to the GUI is implemented on ethernet layer using UDP protocol.
Arduino Ethernet shiled with W5100 is used for this purpose.

To implement pressure regultor with arduino I implemented PID logic using PID library.
One Arduino is going to control two VFDs. So two PID instances were created.
According to the input pressure, this PID logic will calculate the output frequncy for both the VFDs. 

To control the frequency of the motor, the analog port of the VFDs could have been used.
But it requires 0-10V signal level, and I did not wanted to have another voltage regulator.
So I decieded to go with the RS485 communication link between VFD and Arduino using MODBUS protocol.
VFD is from FUJI drives with inbuit MODBUS protocol over RS485 communication link.

FUJI MAKE VFD part Number:
FRN0007C2S-4A

Now the actual flow goes like:
1. Arduino will get pressure on UDP port using ethernet shield.
2. Arduino will process using PID logic and will give two output frequency for two VFDs.
3. These two output frequency will be then sent to the two VFDs using MODBUS protocol on RS485 link.
4. VFDs will be driving at these new frequency.
5. Meanwhile if user wants to monitor or control the VFDs output frequency using GUI. GUI will be asking for the Information from        Arduino using UDP protocol. Arduino will be sending this information to the GUI as well as will stp or run the motor as per           commanded from the GUI

Three GUI has been developed:
1. VFD Controller: To monitor & to control the frequency of the motors. This GUI works on UDP protocol.
2. Pressure Sender Simulator: This GUI only send pressure 


















