/* NeoPixel previews for docs/index.md. Keep the row data in sync with
 * updateLEDS() and tripleFlash() in src/drone.cpp; no drone connection is used.
 */
(() => {
  "use strict";

  function initializePreviews() {
    const toggle = document.getElementById("preview-toggle");
    const status = document.getElementById("preview-status");
    const elements = document.querySelectorAll("[data-led-preview]");
    if (!toggle || !status || elements.length === 0) return;

    const previews = Array.from(elements, (element) => ({
      element,
      pattern: element.dataset.pattern,
      peak: Number(element.dataset.peak),
      onMs: Number(element.dataset.onMs || 0),
      offMs: Number(element.dataset.offMs || 0),
      floor: Number(element.dataset.floor || 0),
      flashCount: Number(element.dataset.flashCount || 1),
      flashGapMs: Number(element.dataset.flashGapMs || 0),
    }));

    // This generates Adafruit_NeoPixel's 8-bit sine envelope. The integer
    // rounding and optional squared curve below follow the firmware.
    const sine8 = Array.from({ length: 256 }, (_, angle) =>
      Math.round((Math.sin((angle * Math.PI) / 128) + 1) * 127.5)
    );
    const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
    let paused = reducedMotion.matches;
    let startTime = performance.now();
    let lastFrame = -Infinity;
    let frameId = null;

    function brightnessAt(preview, elapsedMs) {
      if (preview.pattern === "solid") return preview.peak;

      if (preview.pattern === "breathe") {
        const pulseMs = Math.max(1000, preview.onMs);
        const phase = elapsedMs % (pulseMs + preview.offMs);
        if (phase >= pulseMs) return 0;
        const angle = (192 + Math.floor((phase * 256) / pulseMs)) % 256;
        let envelope = sine8[angle];
        const floor = Math.min(preview.floor, preview.peak);
        if (floor > 0) envelope = Math.floor((envelope * envelope + 127) / 255);
        return floor + Math.floor(((preview.peak - floor) * envelope + 127) / 255);
      }

      const spacing = preview.onMs + preview.flashGapMs;
      const activeMs = preview.flashCount * preview.onMs +
        (preview.flashCount - 1) * preview.flashGapMs;
      const phase = elapsedMs % (activeMs + preview.offMs);
      return phase < activeMs && phase % spacing < preview.onMs ? preview.peak : 0;
    }

    function paint(elapsedMs) {
      for (const preview of previews) {
        const brightness = paused ? preview.peak : brightnessAt(preview, elapsedMs);
        preview.element.style.setProperty("--led-level", String(brightness / 255));
      }
    }

    function animate(now) {
      frameId = null;
      if (paused || document.hidden) return;
      // The firmware refreshes the RGBW chain at most once every 20 ms.
      if (now - lastFrame >= 20) {
        paint(Math.floor(now - startTime));
        lastFrame = now;
      }
      frameId = requestAnimationFrame(animate);
    }

    function refresh() {
      if (frameId !== null) cancelAnimationFrame(frameId);
      frameId = null;
      startTime = performance.now();
      lastFrame = -Infinity;
      toggle.textContent = paused ? "Play animations" : "Pause animations";
      status.textContent = paused
        ? "Paused. Colors are shown at their configured peak brightness."
        : "Playing at firmware timing. Light 1 is on the left.";
      paint(0);
      if (!paused && !document.hidden) frameId = requestAnimationFrame(animate);
    }

    toggle.hidden = false;
    toggle.addEventListener("click", () => {
      paused = !paused;
      refresh();
    });
    reducedMotion.addEventListener("change", (event) => {
      paused = event.matches;
      refresh();
    });
    document.addEventListener("visibilitychange", refresh);
    refresh();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initializePreviews, { once: true });
  } else {
    initializePreviews();
  }
})();
