import json
import os
import re
import sys

def main():
    # Resolve paths relative to this script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    tel_hpp_path = os.path.join(project_root, "hal", "telemetry_registry.hpp")
    json_path = os.path.join(project_root, "app", "telemetry_registry.json")

    if not os.path.exists(tel_hpp_path):
        print(f"Error: Source header file not found at {tel_hpp_path}", file=sys.stderr)
        sys.exit(1)

    with open(tel_hpp_path, "r") as f:
        content = f.read()

    # Regex matching: {"name", id, &::app::ui.field, scale, "unit", "description"}
    # Note: Handles optional f suffix on scale float values, spaces, and fully qualified namespaces
    pattern = re.compile(
        r'\{\s*"([^"]+)"\s*,\s*(\d+)\s*,\s*&[^,]+\s*,\s*([\d\.]+)f?\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*\}'
    )

    telemetry_list = []
    for line in content.splitlines():
        # Skip commented-out lines
        if line.strip().startswith("//"):
            continue
            
        match = pattern.search(line)
        if match:
            name, vid_str, scale_str, unit, desc = match.groups()
            telemetry_list.append({
                "id": int(vid_str),
                "name": name,
                "scale": float(scale_str),
                "unit": unit,
                "description": desc
            })

    if not telemetry_list:
        print("Warning: No telemetry entries parsed from C++ registry file!", file=sys.stderr)

    # Static commands definition that MCU code expects
    commands_list = [
        {"name": "mEnable", "type": "uint8_t", "default": 0},
        {"name": "mMode", "type": "uint8_t", "default": 0},
        {"name": "targetSpeed_rpm", "type": "float", "default": 0.0},
        {"name": "mAcceleration_rpm_s", "type": "float", "default": 500.0},
        {"name": "mIsAbs_mA", "type": "float", "default": 0.0}
    ]

    registry_data = {
        "commands": commands_list,
        "telemetry": telemetry_list
    }

    # Write formatted JSON output
    with open(json_path, "w", newline="\n") as f:
        json.dump(registry_data, f, indent=2)

    print(f"Telemetry registry successfully parsed. Exported {len(telemetry_list)} entries to JSON.")

if __name__ == "__main__":
    main()
