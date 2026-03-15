<h1>STM32 2-Axis Drawing Robot (Bare-Metal Firmware)</h1>
<h2>1. About The Project</h2>

<p>This repository contains the firmware and hardware configurations for a custom-built 2-Axis Drawing Robot. The core system is powered by an STM32F103C8T6 (Blue Pill) microcontroller, programmed entirely in bare-metal C/C++ to ensure maximum performance and precise timing.</p>

<p><strong>Key Technical Highlights:</strong></p>
<ul>
  <li><strong>Firmware Architecture:</strong> Optimized stepper motor control using STM32 Hardware Timers (TIM1 &amp; TIM2) and Interrupts to generate precise, non-blocking step pulses (jitter-free movements).</li>
  <li><strong>Path Planning:</strong> Implemented Bresenham's line algorithm combined with a custom G-code parser to translate standard vector commands into accurate X-Y coordinate steps.</li>
  <li><strong>Hardware Integration:</strong> Calibrated and integrated stepper motor drivers, rigorously validating torque control logic to maintain system stability under mechanical load.</li>
</ul>

<p><strong>Tech Stack:</strong> <code>STM32</code> | <code>C/C++</code> | <code>UART Debugging</code> | <code>Hardware Timers/Interrupts</code></p>

<h2>2. Hardware Requirements (BOM)</h2>

<ul>
  <li>1x <strong><font color="#005cc5">STM32F103C8T6</font></strong> (Blue Pill) Microcontroller</li>
  <li>2x <strong><font color="#005cc5">NEMA 17</font></strong> Stepper Motors</li>
  <li>2x <strong><font color="#005cc5">A4988 / DRV8825</font></strong> Stepper Motor Drivers</li>
  <li>2x <strong><font color="#005cc5">Mechanical Limit Switches</font></strong> (Endstops)</li>
  <li>4x <strong><font color="#005cc5">Push Buttons</font></strong> (for manual UI control)</li>
  <li>1x <strong><font color="#005cc5">12V DC Power Supply</font></strong></li>
  <li>1x <strong><font color="#005cc5">ST-Link V2</font></strong> (For programming and debugging)</li>
</ul>

<h2>3. Circuit &amp; Wiring (Pinout Configuration)</h2>

<p>The system peripherals are mapped directly to the STM32 hardware blocks for optimized DMA/Interrupt handling.</p>

<h3>⚙️ Motor Control (A4988/DRV8825)</h3>

<table>
  <thead>
    <tr>
      <th>STM32 Pin</th>
      <th>Peripheral / Label</th>
      <th>Driver Pin</th>
      <th>Function</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong><font color="#005cc5">PA8</font></strong></td>
      <td><strong><font color="#22863a">TIM1_CH1</font></strong></td>
      <td>STEP (Axis 1)</td>
      <td>PWM/Timer pulse generation for precise stepping</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PB15</font></strong></td>
      <td><strong><font color="#6f42c1">GPIO_Output</font></strong></td>
      <td>DIR1 (Axis 1)</td>
      <td>Direction control for Axis 1</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PB14</font></strong></td>
      <td><strong><font color="#6f42c1">GPIO_Output</font></strong></td>
      <td>ENA1 (Axis 1)</td>
      <td>Enable/Disable driver 1</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA0</font></strong></td>
      <td><strong><font color="#22863a">TIM2_CH1</font></strong></td>
      <td>STEP (Axis 2)</td>
      <td>PWM/Timer pulse generation for precise stepping</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA1</font></strong></td>
      <td><strong><font color="#6f42c1">GPIO_Output</font></strong></td>
      <td>DIR2 (Axis 2)</td>
      <td>Direction control for Axis 2</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA2</font></strong></td>
      <td><strong><font color="#6f42c1">GPIO_Output</font></strong></td>
      <td>ENA2 (Axis 2)</td>
      <td>Enable/Disable driver 2</td>
    </tr>
  </tbody>
</table>

<h3>🛑 Limit Switches &amp; UI Control</h3>

<table>
  <thead>
    <tr>
      <th>STM32 Pin</th>
      <th>Label</th>
      <th>Function</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong><font color="#005cc5">PA6</font></strong></td>
      <td><strong><font color="#6f42c1">StopX</font></strong></td>
      <td>Endstop / Limit Switch for X-Axis home position</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA5</font></strong></td>
      <td><strong><font color="#6f42c1">StopY</font></strong></td>
      <td>Endstop / Limit Switch for Y-Axis home position</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA7</font></strong></td>
      <td><strong><font color="#6f42c1">button1</font></strong></td>
      <td>Manual Control UI</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PB1</font></strong></td>
      <td><strong><font color="#6f42c1">button2</font></strong></td>
      <td>Manual Control UI</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PB10</font></strong></td>
      <td><strong><font color="#6f42c1">button3</font></strong></td>
      <td>Manual Control UI</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PB11</font></strong></td>
      <td><strong><font color="#6f42c1">button4</font></strong></td>
      <td>Manual Control UI</td>
    </tr>
  </tbody>
</table>

<h3>🔌 Communication &amp; Debugging</h3>

<table>
  <thead>
    <tr>
      <th>STM32 Pin</th>
      <th>Peripheral</th>
      <th>Function</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong><font color="#005cc5">PA9</font></strong></td>
      <td><strong><font color="#22863a">USART1_TX</font></strong></td>
      <td>Programming MCU via UART Bootloader (TX)</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA10</font></strong></td>
      <td><strong><font color="#22863a">USART1_RX</font></strong></td>
      <td>Programming MCU via UART Bootloader (RX)</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA13</font></strong></td>
      <td><strong><font color="#22863a">SYS_JTMS-SWDIO</font></strong></td>
      <td>ST-Link Debug Data</td>
    </tr>
    <tr>
      <td><strong><font color="#005cc5">PA14</font></strong></td>
      <td><strong><font color="#22863a">SYS_JTCK-SWCLK</font></strong></td>
      <td>ST-Link Debug Clock</td>
    </tr>
  </tbody>
</table>

<h2>4. Getting Started</h2>

<h3>Prerequisites</h3>
<ul>
  <li><strong>STM32CubeIDE</strong> (or Keil uVision)</li>
  <li><strong>ST-Link V2</strong> USB Programmer</li>
  <li>A Serial Terminal software (e.g., PuTTY, Hercules, or a custom G-code sender)</li>
</ul>

<h3>Installation &amp; Flashing</h3>
<ol>
  <li>Clone this repository:
    <pre><code>git clone https://github.com/your-username/your-repo-name.git</code></pre>
  </li>
  <li>Open the project folder in <strong>STM32CubeIDE</strong>.</li>
  <li>Build the project (<code>Project -&gt; Build All</code>).</li>
  <li>Connect the ST-Link V2 to the STM32 board (<code>SWDIO</code> to <code>PA13</code>, <code>SWCLK</code> to <code>PA14</code>, <code>GND</code>, <code>3.3V</code>).</li>
  <li>Click <code>Run -&gt; Debug</code> to flash the firmware into the MCU.</li>
</ol>

<h2>5. Usage (Manual Control)</h2>
<ol>
  <li>Power up the 12V supply for the stepper motors and the 3.3V/5V supply for the STM32.</li>
  <li>Wait a moment for the system to initialize.</li>
  <li>Use the physical push buttons to automatically draw pre-programmed shapes:
    <ul>
      <li><strong>Button 1 (PA7):</strong> Draw a Square</li>
      <li><strong>Button 2 (PB1):</strong> Draw a Rectangle</li>
      <li><strong>Button 3 (PB10):</strong> Draw a Star</li>
      <li><strong>Button 4 (PB11):</strong> Draw a Circle</li>
    </ul>
  </li>
  <li>The robot will automatically stop moving if it triggers the limit switches (PA5, PA6).</li>
</ol>

<p><i>If you find this project interesting, feel free to give it a ⭐!</i></p>
