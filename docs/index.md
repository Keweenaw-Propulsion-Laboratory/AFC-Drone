---
title: LED Color Codes
description: Identify the AFC Drone's state from its two NeoPixel lights, with animated previews and timing details.
---

<p class="page-eyebrow">AFC DRONE / FLIGHT COMPUTER</p>

# LED Color Codes

The two NeoPixel lights show startup progress, operating states, and faults.
Match the **color and the pattern** to the previews below.

<div class="led-guide">
  <div class="led-order" aria-label="The left dot represents Light 1; the right dot represents Light 2.">
    <span class="led-order__item"><span class="led-order__dot" aria-hidden="true"></span>Light 1</span>
    <span class="led-order__connector" aria-hidden="true"></span>
    <span class="led-order__item"><span class="led-order__dot" aria-hidden="true"></span>Light 2</span>
  </div>
  <p>Previews use chain order: Light 1 is the first pixel connected to the controller; Light 2 follows it. Both lights change together.</p>
</div>

<div class="preview-controls">
  <span id="preview-status" role="status" aria-live="polite">Animated previews use the firmware's timing.</span>
  <button id="preview-toggle" type="button" aria-controls="led-patterns" hidden>Pause animations</button>
</div>

<noscript><p>JavaScript is disabled. Previews show the colors at peak brightness; use the written timing descriptions to identify each pattern.</p></noscript>

<p class="table-scroll-hint">Scroll tables sideways to see timing and meanings.</p>

<div id="led-patterns" markdown="1">

## Startup

Solid colors identify the current initialization stage. Light 1 stays blue while
Light 2 identifies the subsystem being set up.

<div class="led-table-wrap" role="region" aria-label="Startup LED patterns" tabindex="0">
<table class="led-table">
  <caption class="sr-only">Startup states. Preview dots are ordered Light 1, then Light 2.</caption>
  <thead><tr><th scope="col">Preview</th><th scope="col">State</th><th scope="col">Light 1 / Light 2</th><th scope="col">Pattern / timing</th><th scope="col">Meaning</th></tr></thead>
  <tbody>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Both lights solid blue" data-led-preview="BOOT" data-pattern="solid" data-peak="128"><span class="led led--blue"></span><span class="led led--blue"></span></span></td>
      <th scope="row"><code>BOOT</code></th>
      <td>Blue / Blue</td><td><strong>Solid</strong><span class="cell-detail">Continuously lit</span></td>
      <td><strong>Powering up.</strong> Initializes the controller and battery monitor.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Light 1 solid blue, Light 2 solid yellow" data-led-preview="RADIO_SETUP" data-pattern="solid" data-peak="128"><span class="led led--blue"></span><span class="led led--yellow"></span></span></td>
      <th scope="row"><code>RADIO_SETUP</code></th>
      <td>Blue / Yellow</td><td><strong>Solid</strong><span class="cell-detail">Continuously lit</span></td>
      <td><strong>Radio setup.</strong> Resets and configures the RFM69 radio.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Light 1 solid blue, Light 2 solid purple" data-led-preview="SENSOR_SETUP" data-pattern="solid" data-peak="128"><span class="led led--blue"></span><span class="led led--purple"></span></span></td>
      <th scope="row"><code>SENSOR_SETUP</code></th>
      <td>Blue / Purple</td><td><strong>Solid</strong><span class="cell-detail">Continuously lit</span></td>
      <td><strong>Sensor setup.</strong> Initializes the BNO08x inertial measurement unit.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Light 1 solid blue, Light 2 solid white" data-led-preview="CONTROL_SETUP" data-pattern="solid" data-peak="128"><span class="led led--blue"></span><span class="led led--white"></span></span></td>
      <th scope="row"><code>CONTROL_SETUP</code></th>
      <td>Blue / White</td><td><strong>Solid</strong><span class="cell-detail">Continuously lit</span></td>
      <td><strong>Control setup.</strong> Initializes the gimbal and motor outputs, then starts the control timer.</td>
    </tr>
  </tbody>
</table>
</div>

## Operating states

Breathing means a smooth rise and fall in brightness. Flight patterns use short,
synchronized flashes: green on Light 1 and red on Light 2. Both flight patterns
repeat every **0.8 seconds**; manual flight flashes once and autonomous flight
flashes twice.

<div class="led-table-wrap" role="region" aria-label="Operating state LED patterns" tabindex="0">
<table class="led-table">
  <caption class="sr-only">Operating states. Both lights breathe or flash in sync.</caption>
  <thead><tr><th scope="col">Preview</th><th scope="col">State</th><th scope="col">Light 1 / Light 2</th><th scope="col">Pattern / timing</th><th scope="col">Meaning</th></tr></thead>
  <tbody>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Both lights breathing green over four seconds, remaining dimly lit between peaks" data-led-preview="SAFE" data-pattern="breathe" data-peak="128" data-on-ms="4000" data-off-ms="0" data-floor="3"><span class="led led--green"></span><span class="led led--green"></span></span></td>
      <th scope="row"><code>SAFE</code></th>
      <td>Green / Green</td><td><strong>Breathing</strong><span class="cell-detail">4 s rise / fall<br>No dark pause</span></td>
      <td><strong>Disarmed.</strong> Startup is complete. Gimbal and motor movement are disabled.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Both lights breathing yellow over four seconds, remaining dimly lit between peaks" data-led-preview="READY_ARMED" data-pattern="breathe" data-peak="128" data-on-ms="4000" data-off-ms="0" data-floor="3"><span class="led led--yellow"></span><span class="led led--yellow"></span></span></td>
      <th scope="row"><code>READY_ARMED</code></th>
      <td>Yellow / Yellow</td><td><strong>Breathing</strong><span class="cell-detail">4 s rise / fall<br>No dark pause</span></td>
      <td><strong>Armed.</strong> Gimbal movement is allowed with a healthy heartbeat; motors remain at idle.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Light 1 green and Light 2 red: one synchronized 100 millisecond flash, then 700 milliseconds dark" data-led-preview="MAN_FLIGHT" data-pattern="flash" data-peak="255" data-on-ms="100" data-off-ms="700" data-flash-count="1" data-flash-gap-ms="0"><span class="led led--green"></span><span class="led led--red"></span></span></td>
      <th scope="row"><code>MAN_FLIGHT</code></th>
      <td>Green / Red</td><td><strong>Single flash</strong><span class="cell-detail">100 ms on · 700 ms off<br>0.8 s per cycle</span></td>
      <td><strong>Manual flight.</strong> Gimbal and motors follow manual commands while the watchdog is fed.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Light 1 green and Light 2 red: two synchronized 100 millisecond flashes, separated by 100 milliseconds, then 500 milliseconds dark" data-led-preview="AUTO_FLIGHT" data-pattern="flash" data-peak="255" data-on-ms="100" data-off-ms="500" data-flash-count="2" data-flash-gap-ms="100"><span class="led led--green"></span><span class="led led--red"></span></span></td>
      <th scope="row"><code>AUTO_FLIGHT</code><span class="state-tag">Control pending</span></th>
      <td>Green / Red</td><td><strong>Double flash</strong><span class="cell-detail">100 ms on / off / on<br>500 ms dark · 0.8 s cycle</span></td>
      <td><strong>Autonomous flight indication.</strong> The LED pattern is defined, but autonomous flight control is not implemented.</td>
    </tr>
  </tbody>
</table>
</div>

## Faults &amp; link loss

The watchdog indication takes priority over the normal state colors. An amber
pattern can therefore appear while the drone's reported state is <code>SAFE</code>.

<div class="led-table-wrap" role="region" aria-label="Fault and link-loss LED patterns" tabindex="0">
<table class="led-table">
  <caption class="sr-only">Fault and link-loss indications. The watchdog indication overrides state patterns.</caption>
  <thead><tr><th scope="col">Preview</th><th scope="col">State / condition</th><th scope="col">Light 1 / Light 2</th><th scope="col">Pattern / timing</th><th scope="col">Meaning</th></tr></thead>
  <tbody>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Both lights red: two synchronized 100 millisecond flashes, separated by 100 milliseconds, then 500 milliseconds dark" data-led-preview="FAULT_ERROR" data-pattern="flash" data-peak="128" data-on-ms="100" data-off-ms="500" data-flash-count="2" data-flash-gap-ms="100"><span class="led led--red"></span><span class="led led--red"></span></span></td>
      <th scope="row"><code>FAULT_ERROR</code><span class="cell-detail">Also the default pattern</span></th>
      <td>Red / Red</td><td><strong>Double flash</strong><span class="cell-detail">100 ms on / off / on<br>500 ms dark · 0.8 s cycle</span></td>
      <td><strong>Fault.</strong> A startup failure enters the terminal fault state and requires a power cycle. Unrecognized states use the same lights.</td>
    </tr>
    <tr>
      <td><span class="led-pair" role="img" aria-label="Both lights breathing amber for 1.5 seconds, then fully dark for 2.5 seconds" data-led-preview="WATCHDOG" data-pattern="breathe" data-peak="24" data-on-ms="1500" data-off-ms="2500" data-floor="0"><span class="led led--amber"></span><span class="led led--amber"></span></span></td>
      <th scope="row">Watchdog tripped<span class="state-tag state-tag--alert">Overrides state</span></th>
      <td>Amber / Amber</td><td><strong>Breathing + pause</strong><span class="cell-detail">1.5 s rise / fall<br>2.5 s dark · 4 s cycle</span></td>
      <td><strong>Heartbeat timeout.</strong> The watchdog expired while movement was allowed. Outputs fail safe and the vehicle drops to <code>SAFE</code>.</td>
    </tr>
  </tbody>
</table>
</div>

!!! note "The amber indication stays latched"
    A restored link alone does not clear the watchdog alert. The operator must
    release the requested state to `SAFE` before requesting another operating
    state. This prevents an automatic return to flight after a dropout.

</div>

## Reading the previews

- **Order:** left is Light 1, right is Light 2. These labels describe wiring
  order, not a confirmed left/right mounting position on the airframe.
- **Timing:** both lights use the same timing in every pattern shown. A breathing
  duration includes the full rise **and** fall; a double flash ends with the
  listed dark pause after its second flash.
- **Brightness:** startup, armed, disarmed, and fault indications peak at
  `128/255` (about 50%). Flight flashes peak at `255/255`. The watchdog peaks at
  `24/255` (about 9%). Green and yellow breathing stay dimly lit at `3/255` between
  peaks; amber fades fully off.
- **Display:** timing follows the firmware; screen colors, glow, and apparent
  brightness are illustrative. Pause animations to hold every preview at its
  configured peak. Reduced-motion preferences start the page paused.

!!! info "Hardware status"
    The current firmware leaves the NeoPixel output disabled until its data pin
    is assigned. These previews document the configured patterns for that
    two-pixel RGBW chain.

<p class="reference-source">Pattern source: <a href="https://github.com/Keweenaw-Propulsion-Laboratory/AFC-Drone/blob/main/src/drone.cpp"><code>Drone::updateLEDS()</code> and the watchdog indication in <code>drone.cpp</code></a>.</p>
