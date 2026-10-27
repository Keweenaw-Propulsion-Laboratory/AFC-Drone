"""Build LG290P v3.1.1 on Teensy without modifying the downloaded library.

Its unused beginAutoBaudDetect helper calls ESP32-only HardwareSerial methods.
Disable that helper in a generated build copy; GPS uses begin() with an explicit
baud rate. Pin the library version in platformio.ini so this adaptation stays
predictable.
"""

from pathlib import Path

Import("env")


def teensy_lg290p_source(build_env, node):
    source = Path(node.srcnode().get_abspath())
    if source.name != "SparkFun_LG290P_GNSS.cpp":
        return node

    text = source.read_text(encoding="utf-8")
    signature = "bool LG290P::beginAutoBaudDetect("
    if text.count(signature) != 1:
        raise RuntimeError("LG290P compatibility helper requires library v3.1.1")
    start = text.index(signature)
    body_start = text.index("{", start)
    depth = 1
    body_end = body_start + 1
    while depth and body_end < len(text):
        if text[body_end] == "{":
            depth += 1
        elif text[body_end] == "}":
            depth -= 1
        body_end += 1
    if depth:
        raise RuntimeError("Cannot locate LG290P auto-baud helper body")

    stub = """{
    // ESP32-only auto-baud helper is unsupported on Teensy. Use begin().
    (void)serialPort;
    (void)rxPin;
    (void)txPin;
    (void)parserName;
    (void)errorOutput;
    (void)parserDebug;
    (void)debugOutput;
    return false;
}"""
    patched = text[:body_start] + stub + text[body_end:]
    output = Path(build_env.subst("$BUILD_DIR")) / "lg290p_compat" / source.name
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.exists() or output.read_text(encoding="utf-8") != patched:
        output.write_text(patched, encoding="utf-8")
    return build_env.File(str(output))


if env.subst("$PIOPLATFORM") == "teensy":
    env.AddBuildMiddleware(teensy_lg290p_source, "*SparkFun_LG290P_GNSS.cpp")
