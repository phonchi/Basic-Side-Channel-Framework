# Tools

1. aEagle
https://learn.sparkfun.com/tutorials/tags/eagle

> How to design a PCB

2. Baisic about PCB
https://learn.sparkfun.com/tutorials/pcb-basics

> How to read schematic
> How to read datasheet
> How to make PCB
> How to use BOM (bill of material) to assemble board and soldering

3. Baisic about serial comunication
https://learn.sparkfun.com/tutorials/serial-communication

4. Basic electronics
http://www.allaboutcircuits.com/textbook/reference/#chpt-7

> How to use SPICE

5. Basic intergrate circuit
http://www.asic-world.com/verilog/veritut.html
http://www.eda.ncsu.edu/wiki/NCSU_EDA_Wiki
https://www.edaplayground.com/

> How to write hardware circit
> What is FPGA
> How to procceed with EDA flow

### Clock system

### Power system

### The Xilinx ISE Project Management
https://github.com/colinoflynn/makeise

The Xilinx ISE tools use an XML-based project file. Unfortunately it's difficult to commit that project file to the repository - it uses all sorts of automatically overwritten properties, including the version of the software:

![](https://i.imgur.com/sZ4pCnf.png)

What this means is that if you have two people working on the project, you are almost guaranteed to have merge conflicts, since if they have different versions, the file will change on both their machines in that line. Since each install is >10GB people tend to be pretty slow up updating the version of their software. The other issue is that if you want to support several versions of the hardware. This means supporting different FPGA devices with different modules enabled. This would otherwise mean multiple project files, and changes would have to be carefully managed across them. There is even COREGen blocks with are automatically generated, and normally have tons of options you must specify. For someone to port the design to a slightly different FPGA (different package of same FPGA, different device within same family, etc) they must manually recreate all those blocks. 

You can fix this with a bit of Python. Basically there is one '.in' file, which contains simple lines. 

1. The beginning of the file looks like this:

![](https://i.imgur.com/U0Ikvy8.png)

The InputFile is a 'template', which is really just a default Xilinx ISE project file. The next section simply finds parameters and overwrites their values, allowing you to overwrite ISE version, device type, etc. For example if you need to set a specific map command-line option, we can do that as follows:

![](https://i.imgur.com/0Ur7DJd.png)

> Note how the option is set in the resulting .xise project file. Easy! If you want to generate a .xise file for another package, you simply change that one 'package' line as appropriate.

2. Next section of the input file has a list of all the source code. These are inserted into the .xise project file as such:

![](https://i.imgur.com/sFKFpTs.png)

3. We come to the COREGen files. This section can be used like the verilog sources, but the interesting part is the ability to autogenerate COREGen files based on an input. The first section simply lists all the COREGen files:

![](https://i.imgur.com/QcrMiH4.png)

The next sections can list details of the files. For example for ADC FIFO you can configure the depth. Not only does it set the proper depth in the resulting .xco COREGen file, it also calculates other required parameters such as output depth, assert values, etc:

![](https://i.imgur.com/0SNwIQw.png)


4. The resulting .xco files are kept consistent with the device package that was configured earlier in the project file! So you no longer have to manually recreate the .xco COREGen files, they are automatically generated for you based on the single input settings!

![](https://i.imgur.com/JNwIduG.png)

5. The final section of the project file managment is the ability to write a "define" file. If you look back at example Verilog files, there is one (setup.v) that is without a path. But it does have a name, which links to the section name in the project file:

![](https://i.imgur.com/HizyTCh.png)

This will simply create a file called setup.v, where the above will be defined. So the file looks like this:

![](https://i.imgur.com/uNIJwey.png)

That file defines the hardware in use. It is included in each Verilog source file, and if there is hardware-specific configuration it can be managed with simple ``` `ifdef `endif ```sections that include/exclude certain modules. 

The result is end up with a single input file that defines everything about the FPGA project. You can run a Python script which then generates the 'dirty' ISE files. If we need to add a new configuration option to the project, you can trivially add it to the original project file.


### The Python Language

# Target Hardware
## SAKURAG
### Preparation
 
1. A standard USB B-A type cable to connect the SAKURA-G board and a personal computer. 5-V power is supplied from the computer to SAKURA-G through this cable.

2. The  cable  is  used  to  configure  control  and  cryptographic  FGAs  on  SAKURA-G.  Xilinx platform cable USB II is highly recommended. 

3. FTDI Driver: D2XX Driver(http://www.ftdichip.com/Drivers/D2XX.htm)

4. FT_Prog (http://www.ftdichip.com/Support/Utilities.htm)
First time to configure board

5. (Optional )Microsoft.Net Framework 4.0: https://www.microsoft.com/zh-tw/download/details.aspx?id=17718
FTD2XX_NET_DLL (http://www.ftdichip.com/Support/SoftwareExamples/CodeExamples/CSharp.htm)

> 5 is for runnning official checker, and FTD2XX_NET_DLL should be copied into the same directory where SAKURA_Checker.exe is copied. 


### Setup
1. After confirming the power switch SW1 is set to OFF  (center position), connect SAKURA-G (connecter CN6) to PC by using USB cable. In order to configure SPI ROM for the controller FPGA XC6SLX9, connect a configuration cable to the JTAG connector CN4. The board should be kept power-off during the cable connection.

2. In order to enable the USB interface of SAKURA-G, a USB controller chip should be configured using FT_Prog.exe downloaded from FTDI's Website. First, select hardware operation mode "245FIFO" from four modes. Then, select driver mode "D2XX Direct".  The  selection is applied to both port A and B. After  that, write the setting into EEPROM of the USB controller chip. 

3. In order to configure the FPGA, turn on the power witch SW1 (set to the USB side), and execute iMACT software on PC. In the iMPAC, select he following configuration file in the unzipped directory. sakura_g_control /sakura_g_control/ rom_sakura_g_control.mcs. Then select SPI PROM / AT45DB321D, and write the mcs file into PROM. 

4. Similarly, connect SAKURA-G (connecter CN2) to PC after turn off the power of SAKURA-G. Then power on  SAKURA-G and run iMPACT. Select the following configuration file in the unzipped directory. akura_g_aes128/sakura_g_aes128/rom_sakura_g_aes128.mcs. Select SPI PROM / AT45DB321D, and write the mcs file into PROM. 


### Introduction

The SAKURA-G FPGA board is designed for research and development on hardware security, such as Side-Channel Attacks (SCA), Fault Injection Attacks (FIA), Physical Unclonable Functions (PUF), and dynamic reconfiguration. 

Two Spartan™-6 FPGAs are integrated on the board and serve as the controller and main security circuits, respectively. SAKURA-G is highly compatible with SASEBO-GII. 

- Power
The recommended source voltage (EXT 5.0V) range is between 4.75V and 5.25V. The absolute maximum rating of input voltage is 12.6V, limited by amplifier AD8000.

The power supply can be selected to come from the USB B connector (green CN6) or the External 5.0V connector (green EXT. VIN CN1) by toggling Power Switch (green SW1). 
![](https://i.imgur.com/ynGyG40.png)

> The voltage of VCC1V2M (core voltage for the nain FPGA) can be adjusted from 0.5 – 1.5V by using trimmer VR1. Shorting JP2 with a jumper can prevent voltage drop at VCCINT when large power consumption is required. 

![](https://i.imgur.com/mjUNlxz.png)


- Controller
The controller FPGA should be configured before the main FPGA by setting DIP switch SW2 (red, system setting) to an appropriate position, excepting in cases where the main FPGA is configured using a JTAG chain. The configuration mode pins M0 and M1 of the main FPGA can be set using DIP switch SW2 or the controller FPGA. Only DIP switch SW2-1 should be set to high when configuring the main FPGA from the SPI-ROM. 

> JTAG Chain: Each FPGA has its own JTAG chain. The JTAG connectors, CN2 and CN4, are connected to the main FPGA U1 and the controller FPGA U2, respectively. SPI-ROMs are not connected to these JTAG chain. 

- Two measurement points, SMA connectors J1 and J2 are available to monitor power waveforms on the core voltage VCCINT of the main FPGA as shown in Figure. The current-sense resistor R12 can be bypassed by inserting a jumper into JP2. SMA connector J3 outputs the amplified waveform. 

![](https://i.imgur.com/4qDBv4s.png)

- Channel A of the FT2232H is connected to the controller FPGA and channel B is connected to the main FPGA. The reset pin of the FT2232H is simply pulled up. 

![](https://i.imgur.com/0PnqWoe.png)

- Each FPGA has each own on-board 48MHz clock oscillator. The oscillators can be disabled when the FPGAs use external clocks through two pairs of SMA connectors. The USB Interface chip FTDI FT2232H supplies 60MHz clock when the chip is operated in a synchronous FIFO mode. 

- A Spartan™-6 LX75 provides a logic circuit 2.5 times larger than the Virtex™-5 LX30 onSASEBO-GII. 
- Ultra-low-noise board design and an on-board amplifier make power analysis easier. 
 

Users can easily configure and operate SAKURA-G using Verilog-HDL code designed for SASEBO-GII and the free Xilinx® ISE WebPACK design software.

Please visit following shop to order the board. http://www.troche.com/sakura/order.html

# Target Software
### SAKURA-W
### Chipwhisperer
You can use with pip, brew, VM.

# Capture Hardware
## OpenADC with FPGA Controller
![](https://i.imgur.com/beGD6os.png)
It's a simple 2-layer board, and has been carefully routed such that the bottom layer is almost entirely ground plane. There's no separate analog/digital ground, instead the layout tries to keep the analog and digital portions separated such that digital ground currents won't flow over the analog portions. 

> It seemed in my research that separating them can add issues with ground loops when the separation isn't 100% perfect (i.e. you run a digital trace over the analog ground, causing the digital return current to take a much longer path than it would have with a single plane).

The 3.0V analog supply for the ADC comes from an onboard LDO regulator, which filters the 3.3V input supply. The LNA chip required a 5.0V supply so there is also a 3.3V to 5.0V switched-capacitor based DC-DC on board. You'll see a number of ferrite beads (look at the Lx parts) that form supply filters.

OpenADC can work with almost any FPGA board, provided they have something resembling a 2x15 (30-pin) 0.1" header. But it is recommended to use with Xilinx boards, as it makes using the reference design easier. Otherwise you need to modify certain aspects (e.g.: FIFO, DCM blocks).

Follow the https://app.assembla.com/spaces/openadc/wiki/Building_the_OpenADC to build your own one or buy it!

### Spec
- Transformer Input Specifications
Input Impedance: 50 ohm
Ground: Isolated or connected to ADC ground (selectable with solder jumpers)
3db bandwidth: 500 MHz + 

The optional inductor L5 can be used to improve the matching over a specific frequency range. The transformer input by default **does not connect the SMA shield to system ground. This can be used to break ground loops between different pieces of equipment.** 
![](https://i.imgur.com/xv5sd0c.png)



- Low Noise Amplifier Specifications
Input Impedance: 50-ohm or 6k selectable with jumper
Gain Range: -5 dB to 55 dB
Max Input: 3db bandwidth: 120 MHz (note: usable BW higher if signal is pure, as can increase signal gain), 40 MHz analog bandwidth

The AD8331 Low Noise Amplifier is controlled by two pins. One selects the gain mode ‘high’ or ‘low’. The other selects the gain amount. The gain voltage pin is designed to be driven by a PWM signal, and will be filtered on the OpenADC (0.0 to 1.0V). 

> When driving this pin with normal 3.3V logic, the maximum duty cycle of the PWM pin will thus be 30.3%. Do not exceed 2.5V at the AD8331 device or it will be damaged. 

Care must be taken to avoid clipping internally in the AD8331 (LNA) device. The equivalent circuit of the LNA is shown below. 

> Note that the adjustable gain is provided with an attenuator, the result of this is the maximum input power is limited to avoid clipping in the intermediate stages. 

The suggested maximum input signal for the LNA input is -1.8 dBm or 0.635 Vp-p. See AD8331 datasheet for more information.
![](https://i.imgur.com/sIXiSBK.png)
![](https://i.imgur.com/K6WKyZo.png)
![](https://i.imgur.com/M7LjFsS.png)




- ADC Specifications
Max sample rate: 105 MSPS
Bits: 10
Data Format: Parallel
ADC Input Range: Selectable: fixed 2V P-P precision ref, or 1Vp-p - 2Vp-p adjustable

The ADC is configured to output data in ‘offset binary’ mode. See the AD9215 datasheet for more detailed information. The ADC has a ‘duty cycle stabilizer’ enabled by default certain applications which require a changing sample rate may need to disable this feature. The clock must range from 5 to 105 MHz. Parallel termination of the clock line at the ADC is done with a 100-ohm resistor. 

> Note there will be some delay between the FPGA setting the clock high and data being present at the FPGA inputs. The FPGA designer must account for this delay - for example if attempting to clock the external data inputs on the rising edge of the internal FPGA clock, this may fail due to the delay in FPGA output buffers, and the delay in FPGA input buffers. 

> A pad is provided in the ADC clock path which can be connected to a FPGA pin. This would provide a feedback path for the clock - this path will have the same delay as the ADC data signals (output buf + input buf), so data could be clocked on the rising edge of this ‘feedback’ clock. 
 
Capacitors C30 and C31 can be used to provide low pass filtering on the ADC input. They are not mounted by default as they would hamper undersampling applications. The ADC (AD9215) can be configured for a full-scale reading of anywhere from 2V p-p to 1V p-p. 

1. If jumper **JP4** is in the ‘fix’ location the ADC is configured for 2V p-p full-scale using an internal precision reference. 
2. If jumper JP4 is set to the ‘adj’ position, and **R13** is rotated fully clockwise, the ADC is configured for a 1V p-p full scale using an internal precision reference. 
![](https://i.imgur.com/nAZh1Yg.png)


- SMA Connector for external clock input

The ‘Clock In’ can be used to feed an external clock into the FPGA. This pin may alternatively be used for any other purpose - for example feeding a signal out on the SMA connector. Two resistor pads are provided in series & parallel with this SMA connector, which may be used for termination purposes.
![](https://i.imgur.com/mXMI6o5.png)
 

> This signal may not route to a dedicated FPGA clocking resource. In many applications this will be acceptable, since the clock may only be used to generate the sample clock for the ADC. There will be many additional delays which will need to be compensated for in the ADC sample chain, thus the delay added by the non-dedicated clock resource is irrelevant. 
> Note there is no ESD or over-voltage clamping provided, carefully check your host board to see if this should be added.

The trigger input can be used as a trigger input for the system. Note there is no ESD or over-voltage clamping provided, carefully check your host board to see if this should be added.

- Separate voltage regulators for ADC & LNA, 2.25 - 3.6V Digital I/O Compatible

### Header
![](https://i.imgur.com/1ikz2UV.png)

> The ‘No Connection’ pins have no internal connection. When connecting to FPGA boards they may be used as ‘spares’ to route signals onto other pins.

### Controller Overview
The origincal controller is mounted on Spartan 6 LX25-based capture hardware. The smaller Spartan 6 LX9 means the advanced features such as the SAD trigger cannot fit into the FPGA design, but the basic functionality remains.

The design is partitioned into two main clock domains, using appropriate methods for crossing clock domains (dual-port FIFOs, rdy/ack flags). 

1. ADC sampling system domain
2. USB interface domain. 

The ADC domain is constrained to meet 100MHz, and the USB domain is constrained to meet 60MHz. Because several blocks beyond just the FIFO (such as the Sum of Absolute Difference trigger) also need to cross clock domains, and because of the extremely complex dynamic clock routing in this project, the clock domain crossing is of critical importance.

![](https://i.imgur.com/N4JmMgG.png)


The system is designed around a simple 'base' USB communication block. This allows simple reuse of this project for work well outside of embedded hardware security. Adding a block effectively means adding it to the main bus. Each block has an address assigned, and from the (Python) code you can read/write to any addresses exposed by that block. Each 'address' can work in multiple ways: 

1. They can be standard byte-wise registers
2. They can be a FIFO type operation

For example when operating as a FIFO mode, it means if you wrote [0x9A, 0x8F, 0x32] into an address, this has actually loaded those three bytes into a FIFO. Similarly multiple reads from the address unload the FIFO.

![](https://i.imgur.com/Nmv1bFf.png)

#### USB Interface (Rgesiter Control through USB)
A huge portion of this system is USB communications from the PC. The USB block is also the master controller for the bus. The bus was designed to allow maximizing the USB bandwidth, and allows a transfer on each clock cycle. 

> This reduces the speed the FPGA code needs to run at (i.e. you don't need to run at 2x the speed due to wait cycle requirements) while allowing the entire sample buffer to be downloaded over USB.

Iy allows

- Generic USB-Serial chips (CP2102, etc)
- FT2232D/H (High-Speed USB)
- Cypress EZ-USB (High-Speed USB)

From both the FPGA side and the Python API the 'middle ground' of the USB interface is completely hidden. **Thus you can take the same FPGA code and move it from a platform with a high-speed USB to a platform that only has a serial port!** Your transfer speed will be slower, but it's nice having that ability to move things around. 

The main Verilog interface module looks like this, where a small module must interface between your physical link and the 'cmd fifo'. Examples exist in for serial, FTDI chips, and my EZ-USB firmware.

![](https://i.imgur.com/9THeDPP.png)

The API
![](https://i.imgur.com/LWdD8Rs.png)

Where 'mode' is read or write mode, address is the address of the register (or FIFO), and the remaining parameters define data to send or maximum data to return.


### Register Interface
```
input 		reset_i,
/* Fast Clock - ADC Internal Mode. Typically ~100 MHz */
input         clk_adcint,
	 
/* Slower Clock - USB Interface, serial, etc. Typically ~20-60 MHz */
input			clk_iface,
	  
output			clk_adcsample,

/* Interface to computer (e.g.: serial, FTDI chip, etc) */
`ifdef FAST_FTDI	 
/* Interface for FT2232H in Fast Syncronous Mode. Connect the FIFO Clock to 'clk_iface'. */
inout [7:0]  	ftdi_data,
input			ftdi_rxfn,
input			ftdi_txen,
output		 	ftdi_rdn,
output		 	ftdi_wrn,
output		 	ftdi_oen,	
output			ftdi_siwua,
/* Interface for Chipwhisperer Serial port. */
`elsif NEWAEUSBCHIP
inout wire [7:0]	USB_D,
input wire [7:0]	USB_Addr,
input wire		USB_RDn,
input wire		USB_WRn,
input wire		USB_CEn,
input wire		USB_ALEn,
`else
 //Assume serial
input         rxd,
output        txd,	 
`endif	 

/* LEDs. Connect up any you wish. */
output        LED_hbeat, /* Heartbeat LED */
output        LED_armed, /* Armed LED */
output        LED_capture, /* Capture in Progress LED (only illuminate during capture, very quick) */
output        LED_ADCDCMUnlock, /* DCM for ADC is unlocked */
output        LED_CLKGENDCMUnlock, /* DCM for CLKGEN is unlocked */

/* OpenADC Interface Pins */
input [9:0]   ADC_Data,
input         ADC_OR,
output        ADC_clk,
/* Feedback path for ADC Clock. If unused connect to ADC_clk */
input	      ADC_clk_feedback,
input         DUT_CLK_i,
input         DUT_trigger_i,
output        amp_gain,
output        amp_hilo,

/* Generated Clock for other uses */
output 			target_clk


/* Connections to external registers. Use clk_iface for clocking	*/
output		reg_reset_o,
output [5:0]	reg_address_o, 
output [15:0]  reg_bytecnt_o, // Current byte count, starts at 0 for first read/write and increments to reg_size-1
output [7:0]	reg_datao_o,   // Data to write,  When writing data to a register, core must read from this line
input  [7:0]   reg_datai_i,   // Data to read, When reading data from a register, core must write to this line
output [15:0]  reg_size_o,   // Total size being read/write
output		reg_read_o,   // When this line is high on rising edge clk_iface, core should place data read from address specified (and bytecount if used) onto reg_datai. Data is expected to be valid on next rising edge of clock.
output		reg_write_o,  // When this line is high on rising edge of reg_clk, core should write data from reg_datao into specified register address.
output		reg_addrvalid_o, //When this line is high data on reg_address is valid. Expect either reg_read or reg_write to toggle shortly
input		reg_stream_i, //Special-function line that cores can use for more advanced data transfer, including asynchronous and sending data longer than the 65536 maximum the 16-bit byte count gives you. See more details below for useage of this pin.
output [5:0]	reg_hypaddress_o, //Cores must always monitor the reg_hypaddress, and if that address matches one in it's address space, place the size of that register onto the data bus. See details below.
input  [15:0]	reg_hyplen_i,

output [9:0]  ADC_Data_out,
output        ADC_Clk_out

```

> The FIFO interface can be made to work with almost any Computer/FPGA interface with some effort. Examples are provided for a serial port, FTDI FT2232H (and other FTDI chips), and Cypress EZ-USB chip.

(a) Using reg_stream
There are two possible uses for reg_stream:

- Causing data to asynchronously be sent over the computer interface.
- Extending a data transfer beyond the 65536 maximum
 
(1) Sending Data Asynchronously 

As an example of use, the smartcard unit in the 'chipwhisperer' project does this. To send data asynchronously, one places data on the reg_datai bus, and then asserts reg_stream high for a single clock cycle. The data must remain on the reg_datai for two addition clock cycles (e.g.: a total of three). The maximum rate you can send data depends on your underlying interface, you are responsible for somehow knowing this.

When using this mode you should only assert reg_stream if reg_addrvalid, reg_read, and reg_write are all zero. The asynchronous mode can cause problems since if the computer is trying to send data to your core the asynchronous interface will overwrite it. The typical usage of this mode is for low-latency communication of a specific event. Thus the computer would enable the asynchronous mode, and then wait for the core to respond with some event. It will get that response much faster than if it was repetitively polling the core for the event status. Once the event occurs, the core turns off this asynchronous mode.

This can easily be achieved by using one bit in your control register to enable/disable asyncronous mode. For example the Verilog logic to enable the reg_stream pin in the Smartcard controller looks something like this, where scard_async_en comes from a bit of the register:

```
assign reg_stream = scard_async_datardy & scard_async_en & ~(reg_read | reg_write | reg_addrvalid);
```

(2) Extending Data Transfers

When performing a read, the core can assert the reg_stream pin. This must be done sometime AFTER reg_addrvalid goes high, otherwise you will just enable asynchronous mode. Once this mode is enabled, the main control unit will not stop reading once reg_bytecount == reg_size-1, it will continue doing reads until reg_stream is de-asserted. The control software on the computer must be aware of this.

### Clocking System

It uses multiple muxes to select input clocks, switchs different FPGA blocks into the clock path, and routes the resulting clock to multiple possible outputs. 

The clocking system takes available internal/external clocks and uses that to sample the ADC. **By default an "Advanced" clocking module is enabled, which uses features mainly in the Spartan 6 FPGA**. 

As we say there is two input clock sources: the **system clock** and the **external clock**. The system clock runs at some specified frequency - for the LX9 MicroBoard this is 100.00 MHz for example and will be feed to ADC through 23 pin in header. The external clock is input into the OpenADC from SMA.

![](https://i.imgur.com/TMqG7yx.png)

The clock module lets you route a specific clock to the ADC. The most direct routing would take the external clock input and pass it directly to the ADC, with no processing done on it. This has the advantage of being the most flexible: the CLKGEN and DCM blocks add extra features, but also have specific requirements about valid frequency ranges.

> As a warning: the DCM and CLKGEN blocks can become 'unlocked' if the clock falls outside of valid ranges. For example if you have an external clock, stop it, and restart the clock the block will now be unlocked most likely. This will cause the system to fail to capture. The computer interface lets you check the status of these blocks & force them to relock if needed. 

![](https://i.imgur.com/wjxxYNi.png)


(1) Using CLKGEN

The CLKGEN block provides a clock synthesis, which can generate a range of frequencies from either the external or system clock. Be warned the CLKGEN block provides no phase reference between the input and output. If you are using the system clock this doesn't matter, since nothing will be aligned to it anyway. But if you are trying to synchronize samples to an external clock, you cannot insert the CLKGEN block without checking the phase delay.

The input range of the CLKGEN block is specified in the datasheet for the FPGA. For the Spartan 6 devices:

CLKGEN Input: 0.5 - 333 MHz (-2 speed grade) 
CLKGEN Output: 5- 333 MHz (-2 speed grade)

(2) Using the DCM (Phase Adjust)

Unlike the CLKGEN block, the DCM does provide a locked phase reference. In addition you can variably adjust the phase of the signal passing through this block to sample at a specific moment relative to the external clock edge. The DCM block also provides a 1x and 4x clock for convenience you can switch between - useful for oversampling a signal based on an external clock.

Like the CLKGEN block, the DCM has specific frequency ranges you must respect.

Input Range: 5 - 250 MHz (-2 speed grade)
Output Range (1x output): 5 - 250 MHz
Output Range (4x output): 5 - 333 MHz

(3) Note on Allowable Frequency Range

Note you must carefully check all frequencies fall within allowed range. For example you cannot input a 2 MHz clock signal and use the DCM block, since it will fail to lock. You could pass this 2 MHz signal into the CLKGEN block, generate a 6 MHz signal from it, and pass that to the ADC. This means you are oversampling a 3x the clock. 

> In addition the ADC on the OpenADC itself has a lower limit for the sampling rate - thus you cannot pass a slow sample clock to the ADC itself.

The transformer input provides the best wide-band performance. You can use the LNA input at high frequencies - you just may need to add additional gain to compensate for roll-off. The following figure measures a 410 MHz signal using the LNA - notice we need to add 43.7 dB of gain. This will of course amplify low-frequency signals considerably more, so the input spectrum must be very pure (nothing at low frequencies).
![](https://i.imgur.com/m68USPI.png)

#### Synchronous Sampling
One of the core underlying theories is that the measurement of power is **synchronized to the device clock.** This differs from normal oscilloscopes, as they run with an internal timebase. Some scopes will permit you to use an external timebase, but often with a number of caveats, and without providing the clock modification functions (integer multiply/divide, phase shifts).

The ChipWhisperer is designed to interface to a target device, and **use the target device clock to maintain perfect synchronization.** The target device clock can be multiplied, divided, or phase shifted as required. You can read some details of this in a paper on the ADC system, along with the paper on the ChipWhisperer.

It allows you to perform attacks at considerably lower sampling rates. Attacks that were reported to require a 2GS/s oscilloscope can be accomplished with a low-cost ADC when synchronous sampling is used. The following shows a comparison of asynchronous and synchronous sampling, where eight overlapping traces when measured with synchronous sampling show almost zero jitter (in B):

![](https://i.imgur.com/jljVGFc.png)

As an interesting additional feature, you can perform clock recovery on a device running on an internal oscillator. The system to perform this looks something like the following block diagram:

![](https://i.imgur.com/7QQWm2n.png)

Which allows us to detect the fundamental frequency, and then use that frequency for digitizing.

https://app.assembla.com/spaces/chipwhisperer/wiki/Clock_Recovery_Module


### Testing
- Input a sine wave (~1-2 MHz). Make sure it is smooth – if you see spikes you may need to adjust the clock delay.
- Vary the gain, check the voltage at the op-amp pin varies between 0-1V as you adjust this. Measure this at C39/R16.
- Switching between high/low should toggle the voltage on the gainmode op-amp pin, or just see if the gain seems to be switching by looking at the results on-screen
- Connect a logic-level clock input (careful, don’t blow your FPGA up as there is NO BUFFERING). Confirm you can measure the frequency of this clock.
- Switch trigger mode, confirm when you hit ‘capture’ it freezes until you pull trigger high **(hint: you can often just touch it with your hand & noise will pull it high)**

### Probes

#### H-Probe
Rather than inserting a shunt into a circuit, it's also possible to use an H-Field (magnetic field) probe. This allows us to monitor the current with a simple probe placed onto the circuit, possibly even from outside the casing itself. You can purchase such probes, but they tend to be pretty expensive ($500-$2000). 

So instead you can making your own from a length of semi-rigid coax, which you can buy from ebay for a few dollars, or from Digikey for about $10.
https://app.assembla.com/spaces/chipwhisperer/wiki/H-Field_Probe
http://www.digikey.com/product-detail/en/CCSMA-MM-086-6/744-1376-ND/2433485

![](https://i.imgur.com/h5PyAKD.png)

#### Low Noise Amplifier
When using a H-Field probe, one often needs a low noise amplifier. Commercial LNA designs can be at least $100. Again you can build your own which looks something like this:

![](https://i.imgur.com/hooUtgc.png)

Which for as implemented has the following gain & input match parameters (S21/S11):

![](https://i.imgur.com/mtfFu4R.png)
https://app.assembla.com/spaces/chipwhisperer/wiki/Low_Noise_Amplifier_Assembly

#### Differential Probe
Another useful piece of equipment is a differential probe - which again can be very expensive ($1000+). We can use about $25 worth of parts to build our own, which works 'well enough' for most differential power analysis attacks:

![](https://i.imgur.com/tybrgWz.png)
https://app.assembla.com/spaces/chipwhisperer/wiki/Differential_Probe_Assembly



# Capture Software
To communicate with the SAKURA-G board, you can use either the acquisition software provided by chipwhisperer or SAKURA, or design your own communication software. 

To communicate with a PC, the SAKURA-G board embedded an FTDI FT2232D chip. This chip offers two communication channel: A and B. Only the B channel is connected to the control FPGA, so we have to use this channel. When the board is plugged in, two communication ports appears on the PC (you do not need driver if you use Linux, but you need VCP drivers if you use Windows).

Under Linux, the channel A is accessible with the /dev/ttyUSBi device (e.g. /dev/ttyUSB0) and the channel B is accessible with the /dev/ttyUSB(i+1) device (e.g. /dev/ttyUSB1). You can verify the exact device numbers using the command ```dmesg```. Under Windows, if you use the VCP driver, the channel A is accessible with the COMi port (e.g. COM5) and the channel B is accessible with the COM(i+1) port (e.g. COM6).

### Setup

You can change the driver of USB by utilizing http://zadig.akeo.ie/

### General Information 

The traces data is captured to the tmp directory if no project was created. In this case, the "Consolidate" option in the Project menu can be used to copy/move this files to the current Project directory as soon as you create one. This option may also be usefull if you manually added existing traces (editing the .cwp file or using the import option in the Trace Manager) to the project. **Settings are not saved with the project, but instead, you can use the "Save Settings" option (inside the Project menu) to dump this information to the "settings.cwset" file, inside the project directory.** Once saved, the settings can be re-loaded using the load button in the settings tabs (general, scope, traget,...). Another possibility would be to use the Save Button and choose different filenames for each settings profile that you want to use.

The general architectural idea is to start the Flow with a TraceSource, allow the connection of multiple preprocessing modules (that are both: ActiveTraceObserver and TraceSource) and end the chain in an TraceObserver (that can be a widget or an AnalysisSource). The sigTracesChanged signal (in the TraceSource objects) is used to propagate this notification through the chain and activate a callback method (processTraces) in the ActiveTraceObserver objects (like the WaveFormWidget) to read the new data. The PassiveTraceObservers (like the AnalysisSource and the TraceRecorder widget) will ignore this signal, so the processSignal() should be called manually.

In the diagram bellow, the arrows show the data flow (the requests are made in the opposite direction) and the vertical lines the inheritance structure:

![](https://i.imgur.com/J1v6Z7C.png)


- AnalysisSource - define signals (started, updated and done) that the AnalysisObserver objects will "listen" when emitted by these objects to execute the appropriate operation.

- AttackObserver - specialized class that will listen for AttackBaseClass emitted signals and update the ResultWidgets (table, plots...).

- ResultsBase - have a list of registered objects. When changes in this list happen, a signal is emitted to notify the GUI to create a new dock and add it to the Window menu. This list is also largely used in the current code base to allow easy access to the existing widgets (if they exist).

- Plugin - marker interface that informs that the class should be considered when importing plugins from a module.

- AutoScript - helper class with methods to setup the output when the class that uses it need to have scripts as output.

- Parameterized - helper class with methods to maintain (add/find/delete) the parameters when the class that extends it makes use of parameters.

![](https://i.imgur.com/wclIV8j.png)

### Script
User scripts allows partial (i.e.: setting up the environment) or total automation of the execution flow.

A basic script would look like this:
```
from chipwhisperer.common.scripts.base import UserScriptBase


class UserScript(UserScriptBase):
    _name = "ChipWhisperer-Lite: AES SimpleSerial on XMEGA"
    _description = "SimpleSerial with Standard Target for AES (XMEGA)"

    def __init__(self, api):
        super(UserScript, self).__init__(api)

    def run(self):
        #User commands here
        self.api.setParameter(['Generic Settings', 'Scope Module', 'ChipWhisperer/OpenADC'])
        self.api.setParameter(['Generic Settings', 'Target Module', 'Simple Serial'])
        self.api.setParameter(['Generic Settings', 'Trace Format', 'ChipWhisperer/Native'])
        self.api.setParameter(['Simple Serial', 'Connection', 'ChipWhisperer-Lite'])
        self.api.setParameter(['ChipWhisperer/OpenADC', 'Connection', 'ChipWhisperer-Lite'])
                
        self.api.connect()
        
        #Example of using a list to set parameters. Slightly easier to copy/paste in this format
        lstexample = [['CW Extra Settings', 'Trigger Pins', 'Target IO4 (Trigger Line)', True],
                      ['CW Extra Settings', 'Target IOn Pins', 'Target IO1', 'Serial RXD'],
                      ['CW Extra Settings', 'Target IOn Pins', 'Target IO2', 'Serial TXD'],
                      ['OpenADC', 'Clock Setup', 'CLKGEN Settings', 'Desired Frequency', 7370000.0],
                      ['CW Extra Settings', 'Target HS IO-Out', 'CLKGEN'],
                      ['OpenADC', 'Clock Setup', 'ADC Clock', 'Source', 'CLKGEN x4 via DCM'],
                      ['OpenADC', 'Trigger Setup', 'Total Samples', 3000],
                      ['OpenADC', 'Trigger Setup', 'Offset', 1250],
                      ['OpenADC', 'Gain Setting', 'Setting', 45],
                      ['OpenADC', 'Trigger Setup', 'Mode', 'rising edge'],
                      #Final step: make DCMs relock in case they are lost
                      ['OpenADC', 'Clock Setup', 'ADC Clock', 'Reset ADC DCM', None],
                      ]
        for cmd in lstexample: self.api.setParameter(cmd)
        
        #Let's only do a few traces
        self.api.setParameter(['Generic Settings', 'Acquisition Settings', 'Number of Traces', 50])
                      
        #The environment is already set, lets do our first capture
        self.api.capture1()
```

User scripts should inherit from UserScriptBase that specifies the run() method that is correspond to click it in the menu or pressing the attack button. The API is passed as an argument by the GUI through the constructor in order to allow the script to "remote control" the existing section. A name and a description should also be specified.

#### Running from the Terminal
If you want to run the script from the terminal. In this case, you don't need to use the GUI, the capture can be performed using only the API. In order to do it, you should add the following lines to the end of your script file:
```
if __name__ == '__main__':
    from chipwhisperer.common.api.CWCoreAPI import CWCoreAPI
    api = CWCoreAPI()               # Instantiate the API
    api.runScriptClass(UserScript)  # Run UserScript through the API
```
 with GUI
 
 ```
 if __name__ == '__main__':
    from chipwhisperer.common.api.CWCoreAPI import CWCoreAPI
    import chipwhisperer.capture.ui.CWCaptureGUI as cwc       # Import the ChipWhispererCapture GUI
    from chipwhisperer.common.utils.parameter import Parameter
    app = cwc.makeApplication()  
    Parameter.usePyQtGraph = True 
    api = CWCoreAPI()               # Instantiate the API
    gui = cwc.CWAnalyzerGUI(api)    # Instantiate the GUI
    gui.show() 
    api.runScriptClass(UserScript)  # Run UserScript through the API

    app.exec_()
 ```

#### Adding user scripts to the GUI menu
Scripts can be added to the tool menu automatically by saving it in its respective script folder inside the chipwhisperer installation folder or user projects folder:

```
chipwhisperer/software/chipwhisperer/capture/scripts
chipwhisperer/software/chipwhisperer/analyzer/scripts
~/chipwhisperer_projects/chipwhisperer/capture/scripts
~/chipwhisperer_projects/chipwhisperer/analyzer/scripts
```

Files put in these directories are scanned during the GUI initialization and all UserScriptBase classes are added to the menu. You can copy and past the content of the Analysis Script window to a text editor or use the Attack Script Generator->Attack Script->Copy option.

Scripts auto-generated by the analyzer tool can also be executed standalone or saved into the scripts directory so that it will show up in the next execution.

#### Plugin

In the plugin architecture, all modules are scanned during the tool initialization, so new functionalities can be added by just dropping its file inside the respective folder:

```
chipwhisperer/software/chipwhisperer/common/results
chipwhisperer/software/chipwhisperer/common/traces
chipwhisperer/software/chipwhisperer/capture/acq_patterns
chipwhisperer/software/chipwhisperer/capture/auxiliary
chipwhisperer/software/chipwhisperer/capture/scopes
chipwhisperer/software/chipwhisperer/capture/scripts
chipwhisperer/software/chipwhisperer/capture/targets
chipwhisperer/software/chipwhisperer/analyzer/attacks
chipwhisperer/software/chipwhisperer/analyzer/preprocessing
chipwhisperer/software/chipwhisperer/analyzer/scripts
and some of the above subfolders.
```

These paths are checked both inside the tool's root directory (where the tool is installed), and in the Project Folder (default is ~/chipwhisperer_projects), allowing the usage of custom modules without the requirement of being system administer. The CW tools scan these directories looking for classes that inherits from the Plugin class (a marker interface actually) in each public module (that doesn't begin with "_").

> Accessing "Help->List Enabled/Disabled Plugins" in the tool's menu you'll find a list with all modules it tried to load. In case of any problem, you can check in the table the error message and its details. It should be easier to visuallise if you copy and paste the text to a text editor (example: notepad).

These folders usually have a file called base.py or _base.py that contains the base class to all plugins in these directories. Ex:
```
from .base import PreprocessingBase
from chipwhisperer.common.utils.pluginmanager import Plugin


class AddNoiseRandom(PreprocessingBase, Plugin):
    _name = "Add Noise: Amplitude"
    _description = "Add random noise"
     
```
#### Add paramters or options
Parameters are used to allow easy access and manipulation of all object's main attibutes and actions. All parameters can be accessed anywhere in the code throught the Parameter class. It means that if you want to set/get any parameter, you can do it easily adding the follow lines to your code:
```
from chipwhisperer.common.utils.parameter import Parameter

Parameter.setParameter([path,..., value])
value = Parameter.getParameter([path,...])
```

or, if you have access to the api:

```
api.setParameter([path,..., value])
value = api.getParameter([path,...])
```

The easiest way to add parameters to your class, is to make it Parameterized (extending this class). It is an abstract class that declares a public interface and implements two manipulation methods to create/get the parameters and find it. As a general rule, you just need to:

1. Import the Parameterized class: from ```chipwhisperer.common.utils.parameter import Parameterized```

2. Make your class extend it (no construtor call is needed here since the idea is to use it as an interface to avoid the problems with multiple inheritance - i.e.: the diamont one)

3. Define a _name and a _description

4. Register it if it is not readily accessible through a higher parameter hierarchy: ```self.getParams().register()```

5. Call self.getParams().addChildren([...])

The getParams() method does four things:

1. Create a new Parameter if it doesn't exist;
2. Create a group called ```_name```;
3. Create a child description parameter with the  specified _description label;
4. Return a reference to the parent group parameter.

Search is performed using the findParam([fullpath]) method.

Each parameter stores the data internally or externally, using a set/get pair - usefull to retrieve dynamic data. In this case, the @setupSetParam(nameOrPath) decorator should be used in the set method in order to syncronize the GUI when the method is called directly without using the parameter class. More information about the Parameterized and the Parameter class can be found in its docstrings.

Basic example:
```
from chipwhisperer.common.utils.parameter import Parameterized, setupSetParam


class ResultsSave(Parameterized):
    _name = "Save to Files"
    _description = "Save correlation output to files."

    def __init__(self):
        # self.getParams().register()
        self.getParams().addChildren([
            {'name':'Save Raw Results', 'type':'bool', 'get':self.getEnabled, 'set':self.setEnabled},  # With value saved externally
            {'name':'Symbol', 'type':'list', 'values':['o', 's', 't', 'd', '+'], 'value':'o'},         # With value saved internally
        ])

        self.findParam("Symbol").setValue('t')
        s = self.findParam("Symbol").getValue()  # s = 't'

    def getEnabled(self):
        return self._enabled

    @setupSetParam("Save Raw Results")
    def setEnabled(self, enabled):
        self._enabled = enabled
```
# Analysis Software

# Resource
## Traces



# Credit and Reference
Chipwhisperer: http://newae.com/
SAKURA: http://satoh.cs.uec.ac.jp/SAKURA/


