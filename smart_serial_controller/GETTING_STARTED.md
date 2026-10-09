# Getting started: how to start the project, step by step

Follow these steps in order. Each step tells you what success looks like
before you move on. The full reference is in [README.md](README.md).

| Step | What you do | Board needed? | Time |
|---|---|---|---|
| 1 | Put the files on your PC | no | 5 min |
| 2 | Install the tools (once) | no | 1–2 h (download) |
| 3 | Simulate: prove the Verilog works | no | 15 min |
| 4 | Way 1: FPGA only, type `t` and see the temperature | yes | 30 min |
| 5 | Way 2: ARM + FPGA with your C program | yes | 1 h |
| 6 | Verify and collect results for the report | yes | as needed |

---

## Step 1: Put the files on your PC

1. Unzip `smart_serial_controller.zip` into a **short folder with no spaces**,
   for example `C:\ssc\`. You should then have `C:\ssc\smart_serial_controller\`
   with the folders `rtl`, `sim`, `vivado`, `sw` and the others.

   > Vivado has problems with long paths and with spaces in folder names. Do
   > not use `C:\Users\My Name\Downloads\...` or OneDrive folders.

2. Or download the files from GitHub instead: open the branch
   `claude/wizardly-cori-h3t0hy` of your repository, click **Code → Download
   ZIP**, unzip it, and copy the `smart_serial_controller` folder to `C:\ssc\`.

What the folders contain:

```
smart_serial_controller\
  GETTING_STARTED.md   <- this file
  README.md            <- full guide and troubleshooting
  windows\             <- double-click scripts (Windows)
  rtl\                 <- the Verilog design (13 files)
  boards\zedboard\     <- ZedBoard top levels and pin constraints (.xdc)
  sim\                 <- testbenches and fake devices
  vivado\              <- Tcl build scripts (used by the .bat files)
  sw\                  <- C program for the ARM (Vitis)
  docs\                <- code explanation, register map, expected simulation output
```

---

## Step 2: Install the tools (once)

1. **Vivado and Vitis.** Download the *AMD Unified Installer* from the AMD
   website, which needs a free AMD account. Choose the product **Vitis**,
   which installs both Vivado and Vitis. Use any version from 2020.2 to
   2025.x, and install it to the default folder.
   - Under devices you only need **SoCs → Zynq-7000**. Untick the others to
     save disk space.
   - Make sure **Install Cable Drivers** is ticked.
   - The free licence covers the ZedBoard's XC7Z020, so no licence file is
     needed.
2. **ZedBoard board files.** This is only needed for Step 5, but do it now.
   Start Vivado, then go to **Tools → Vivado Store… → Boards**, click
   **Refresh**, open **Avnet**, then **ZedBoard → Install**.
3. **A serial terminal.** Install Tera Term or PuTTY.
4. **On Linux** (instead of Windows), also run the cable-driver installer
   once as root:
   `<Vivado>/data/xicom/cable_drivers/lin64/install_script/install_drivers/install_drivers`

---

## Step 3: Simulate, with no board needed

This proves the Verilog works before you touch hardware.

**Windows:** double-click **`windows\1_simulate.bat`**.

> The first time, Windows may show "Windows protected your PC". Click *More
> info → Run anyway*. If the script prints "Could not find Vivado", open
> `windows\_find_vivado.bat` in Notepad and set `MY_VIVADO_SETTINGS` to your
> `...\Vivado\<version>\settings64.bat`.

**Linux:**

```
cd ~/ssc/smart_serial_controller
source /tools/Xilinx/Vivado/<version>/settings64.sh
vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_ssc_top
vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_ssc_axi
vivado -mode batch -source vivado/run_sim.tcl -tclargs tb_zed_standalone
```

**Success looks like this.** Each of the three runs ends with:

```
==============================================================
 ALL 411 CHECKS PASSED          (tb_ssc_top)
 ALL 9 CHECKS PASSED            (tb_ssc_axi)
 ALL 10 CHECKS PASSED           (tb_zed_standalone)
==============================================================
```

Compare your output with the expected logs in `docs\sim_results\`.

**To see waveforms** (for your report screenshots):

1. Run `windows\2_build_way1_fpga_only.bat` once. It also creates the
   project.
2. Run `windows\4_open_project_gui.bat`.
3. In *Sources → Simulation Sources → sim_1*, right-click
   `tb_ssc_top` and choose **Set as Top**.
4. Click **Run Simulation → Run Behavioral Simulation**.
5. Type `run all` in the Tcl Console at the bottom.
6. Drag signals such as `scl`, `sda` and `dut/u_i2c/st` into the waveform
   window.

---

## Step 4: Way 1, FPGA only

### 4a. Build the bitstream

Double-click **`windows\2_build_way1_fpga_only.bat`** and wait 5–15 minutes.

**Success:** the script prints `worst setup slack (WNS) : <number> ns` and the
number is 0 or positive. The bitstream is
`build\standalone\zed_top_standalone.bit`.

### 4b. Prepare the board

1. **Boot-mode jumpers:** set JP7, JP8, JP9, JP10 and JP11 all to the **GND**
   side. This is JTAG mode.
2. **Plug in the Pmods with the power OFF.** Pin 1 of each Pmod is marked
   on the board.
   - **JA:** PmodUSBUART, top row.
   - **JB:** PmodSF3, the whole 12-pin connector.
   - **JC:** PmodTMP2. Put its 2×4 header in the **right-hand four
     columns**, pins 3–6 and 9–12, so that SCL = JC3 and SDA = JC4. Leave
     both address jumpers on the PmodTMP2 **open**, which gives address
     0x4B.
3. Connect the cables:
   - **J17 (PROG)** micro-USB to the PC.
   - The **PmodUSBUART** micro-USB to the PC.
   - The 12 V supply.
4. Switch the board on. The green POWER LED lights.

### 4c. Program the FPGA

1. Open Vivado (the GUI).
2. Click **Open Hardware Manager → Open Target → Auto Connect**.
3. Click **Program Device**, browse to
   `build\standalone\zed_top_standalone.bit`, and click **Program**.
4. The blue **DONE** LED lights, and LD0 starts blinking.

### 4d. Talk to it

1. Open **Device Manager → Ports (COM & LPT)**. The PmodUSBUART shows up as
   **USB Serial Port (COMx)**.
2. In Tera Term or PuTTY, open that COM port at **115200 baud, 8 data bits,
   no parity, 1 stop bit, no flow control**.
3. Press **BTNC** (the centre push button) on the ZedBoard.

**Success:**

```
Smart Serial Controller - Way 1 bring-up
  t = temperature (PmodTMP2 on JC)
  f = flash JEDEC ID (PmodSF3 on JB)
> t
Temp = +24.6875 C
> f
Flash ID = 20 BA 19
```

Touch the sensor with your finger and press `t` again: the temperature
should rise. If something is wrong, see section 7 of the README.

---

## Step 5: Way 2, ARM + FPGA (the final system)

### 5a. Build the hardware

Double-click **`windows\3_build_way2_arm_fpga.bat`** and wait 10–20 minutes.

**Success:** the script prints the path of
`build\ps_system\ssc_system.xsa`. This is the hardware description that Vitis
needs.

To see the block design (the architecture slide drawn by Vivado), open
`build\ps_system\ssc_ps.xpr` in Vivado and choose **Open Block Design**.

### 5b. Create the software in Vitis

**Vitis 2023.2 or newer (Unified IDE):**

1. Start Vitis and choose an empty workspace folder, for example
   `C:\ssc\vitis_ws`.
2. Go to **File → New Component → Platform**:
   - Name it `ssc_platform`.
   - For the hardware design, browse to `build\ps_system\ssc_system.xsa`.
   - Choose standalone, ps7_cortexa9_0, then **Finish**.
   - Click **Build** in the Flow panel.
3. Go to **File → New Component → Application**:
   - Name it `ssc_app`, select `ssc_platform`, then **Finish**.
4. Copy the four files from `sw\` (`main.c`, `ssc_driver.c`, `ssc_driver.h`,
   `ssc_regs.h`) into `ssc_app\src`.
5. Click **Build**.

**Vitis 2020.2 – 2023.1 (Classic):**

1. Go to **File → New → Application Project → Next**.
2. Choose **Create a new platform from hardware (XSA)** and browse to the
   `.xsa` file.
3. Name the project `ssc_app` and select processor `ps7_cortexa9_0`.
4. Choose the template **Empty Application (C)**, then **Finish**.
5. Right-click `ssc_app/src` and choose **Import Sources**. Pick the `sw`
   folder and all four files.
6. Click the hammer icon to build.

### 5c. Run

1. Connect the **J14 (UART)** micro-USB as well. This is the ARM's console
   port; it shows up as a second COM port.
2. Open a terminal on that COM port at **115200 8N1**.
3. Start the program:
   - **Unified IDE:** click **Run** in the Flow panel.
   - **Classic:** right-click `ssc_app` and choose **Run As → Launch
     Hardware**.

   Vitis loads the bitstream and starts your program.

**Success:**

```
=== Smart Serial Controller - Way 2 (ARM + FPGA) ===
controller found at 0x43c00000
ssc> selftest        -> all PASS (works even with no Pmods)
ssc> temp            -> +24.xxxx C
ssc> id              -> 20 ba 19
ssc> flash           -> read back 256 bytes: 0 errors PASS
ssc> bench           -> CPU time with and without the FIFO
```

---

## Step 6: Verify and collect results

Use the checklist in **README section 6**. It has four levels:

1. **Simulation:** the PASSED messages, plus waveform screenshots.
2. **Way 1 on the board.**
3. **Way 2 on the board.**
4. **Logic analyser on JD:** the I2C capture must match slide 13.

For the report, collect these numbers:

| Number | Where it comes from |
|---|---|
| Utilisation | `build\standalone\utilization.rpt` |
| Timing (WNS) | `timing_summary.rpt` |
| Baud error table | test [9] in the simulation log |
| CPU time saved | the `bench` command |
| Bytes moved by the bridge | `bridge off` |

---

## How to learn the code (suggested reading order)

1. [docs/CODE_WALKTHROUGH.md](docs/CODE_WALKTHROUGH.md), the explanation of
   every file.
2. The RTL, in this order (each file starts with a comment block):
   1. `ssc_sync_filter.v`
   2. `ssc_fifo.v`
   3. `ssc_baud_gen.v`
   4. `ssc_uart.v`
   5. `ssc_spi.v`
   6. `ssc_i2c.v`
   7. `ssc_bridge.v`
   8. `ssc_irq.v`
   9. `ssc_regbank.v`
   10. `ssc_apb_top.v`
   11. `ssc_axi_apb_bridge.v`
   12. `ssc_axi_top.v`
   13. `ssc_cmd_fsm.v`
3. [docs/REGISTER_MAP.md](docs/REGISTER_MAP.md) before you change the C
   code.
4. `sim/tb_ssc_top.v`: each test task `t_...` shows how to use one feature
   from software.

## How this fits your 14-week plan

| Weeks | Slide stage | In this folder |
|---|---|---|
| 1–3 | Spec + RTL | `rtl\` (done; study and explain it) |
| 3–7 | Simulation | `sim\` (Step 3) |
| 7–9 | Synthesis and timing | Step 4a / 5a reports |
| 9–12 | ZedBoard bring-up | Steps 4–5 |
| 12–14 | Results and report | Step 6 |
