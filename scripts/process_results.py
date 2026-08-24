#!/usr/bin/env python3
"""
Quad ESP32-WROOM RF Antenna Testing System
Linux PC Result Integration, Processing, and Visualization Script

This script communicates over USB Serial with the Reference Node Master ESP32,
captures DATA log lines, parses RSSI, Packet Error Rates (PER), and CSI subcarrier
complex I/Q data, exports results to CSV/JSON, and generates visualization plots.
"""

import sys
import os
import time
import re
import math
import json
import csv
import argparse

try:
    import serial
except ImportError:
    serial = None

# Optional plotting with matplotlib
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


PHASE_NAMES = {
    0: "REF_V_TX",
    1: "REF_H_TX",
    2: "TEST_A_TX",
    3: "TEST_R_TX"
}


def parse_data_line(line):
    """
    Parses a single DATA line from the Reference Node Master.
    Example line:
    DATA,run=1,test=1,ch=1,phase=0,node=REF_MASTER,ant=REF_V_ANT,mode=2,frames=0,pkts=200,match=200,rssi_avg=0.00,rssi_std=0.00,min=127,max=-127,csi_i=[0;0;...],csi_q=[0;0;...]
    """
    line = line.strip()
    if not line.startswith("DATA,"):
        return None

    record = {}

    # Extract csi_i and csi_q arrays
    csi_i_match = re.search(r',csi_i=\[([^\]]*)\]', line)
    csi_q_match = re.search(r',csi_q=\[([^\]]*)\]', line)

    if csi_i_match:
        raw_i = csi_i_match.group(1)
        record['csi_i'] = [int(x) for x in raw_i.split(';') if x.strip() != ''] if raw_i else []
    else:
        record['csi_i'] = []

    if csi_q_match:
        raw_q = csi_q_match.group(1)
        record['csi_q'] = [int(x) for x in raw_q.split(';') if x.strip() != ''] if raw_q else []
    else:
        record['csi_q'] = []

    # Strip CSI portions to parse key-value pairs
    clean_line = re.sub(r',csi_i=\[[^\]]*\]', '', line)
    clean_line = re.sub(r',csi_q=\[[^\]]*\]', '', clean_line)

    parts = clean_line.split(',')
    for part in parts[1:]:
        if '=' in part:
            k, v = part.split('=', 1)
            k = k.strip()
            v = v.strip()
            if k in ['run', 'test', 'ch', 'phase', 'mode', 'frames', 'pkts', 'match', 'min', 'max']:
                record[k] = int(v)
            elif k in ['rssi_avg', 'rssi_std']:
                record[k] = float(v)
            else:
                record[k] = v

    # Compute CSI magnitude per bin
    csi_mag = []
    for real, imag in zip(record.get('csi_i', []), record.get('csi_q', [])):
        mag = math.sqrt(real * real + imag * imag)
        csi_mag.append(round(mag, 2))
    record['csi_mag'] = csi_mag

    # Compute Packet Loss Rate / Error Rate if RX mode
    pkts = record.get('pkts', 0)
    match = record.get('match', 0)
    if record.get('mode') == 1 and pkts > 0: # RX
        record['per'] = round(1.0 - (match / float(pkts)), 4)
    else:
        record['per'] = 0.0

    return record


def process_serial(port, baudrate, output_csv, output_json, duration=None):
    """
    Reads from Serial port and processes incoming data lines.
    """
    if serial is None:
        print("Error: pyserial package is required for live serial collection. Install with 'pip install pyserial'.")
        sys.exit(1)

    print(f"Opening Serial port {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1.0)
    time.sleep(2) # Allow ESP32 reboot / serial stabilization

    records = []
    start_time = time.time()

    print("Listening for RF sweep data... (Press Ctrl+C to stop)")
    try:
        while True:
            if duration and (time.time() - start_time) > duration:
                break

            line = ser.readline().decode('utf-8', errors='ignore')
            if not line:
                continue

            sys.stdout.write(line)
            sys.stdout.flush()

            record = parse_data_line(line)
            if record:
                records.append(record)

    except KeyboardInterrupt:
        print("\nStopping serial capture.")
    finally:
        ser.close()

    save_records(records, output_csv, output_json)
    return records


def parse_file(input_file, output_csv, output_json):
    """
    Parses a log file containing raw USB serial output.
    """
    print(f"Reading log file: {input_file}")
    records = []
    with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            rec = parse_data_line(line)
            if rec:
                records.append(rec)

    print(f"Parsed {len(records)} data records.")
    save_records(records, output_csv, output_json)
    return records


def save_records(records, output_csv, output_json):
    if not records:
        print("No valid records to save.")
        return

    # Export to JSON
    if output_json:
        with open(output_json, 'w', encoding='utf-8') as f:
            json.dump(records, f, indent=2)
        print(f"Saved JSON results to {output_json}")

    # Export to CSV
    if output_csv:
        fieldnames = [
            'run', 'test', 'ch', 'phase', 'node', 'ant', 'mode', 'frames',
            'pkts', 'match', 'per', 'rssi_avg', 'rssi_std', 'min', 'max',
            'csi_i', 'csi_q', 'csi_mag'
        ]
        with open(output_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in records:
                row = r.copy()
                row['csi_i'] = ';'.join(map(str, row.get('csi_i', [])))
                row['csi_q'] = ';'.join(map(str, row.get('csi_q', [])))
                row['csi_mag'] = ';'.join(map(str, row.get('csi_mag', [])))
                writer.writerow(row)
        print(f"Saved CSV results to {output_csv}")


def generate_plots(records, output_prefix="rf_antenna_test"):
    if not HAS_MATPLOTLIB:
        print("Matplotlib not installed. Skipping plot generation.")
        return

    if not records:
        print("No records available to plot.")
        return

    print("Generating analysis plots...")

    # Group by channel and antenna
    channels = sorted(list(set(r['ch'] for r in records if 'ch' in r)))

    # 1. RSSI vs Channel for Test Antenna
    plt.figure(figsize=(10, 6))
    for ant in set(r['ant'] for r in records if 'ant' in r):
        ant_records = [r for r in records if r.get('ant') == ant and r.get('mode') == 1] # RX mode
        if not ant_records:
            continue

        ch_rssi = {}
        for r in ant_records:
            ch = r['ch']
            ch_rssi.setdefault(ch, []).append(r['rssi_avg'])

        x_vals = sorted(ch_rssi.keys())
        y_vals = [sum(ch_rssi[ch]) / len(ch_rssi[ch]) for ch in x_vals]
        plt.plot(x_vals, y_vals, marker='o', label=f"Antenna: {ant}")

    plt.title("Average Received RSSI vs Wi-Fi Channel")
    plt.xlabel("Wi-Fi Channel")
    plt.ylabel("RSSI (dBm)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    rssi_plot_path = f"{output_prefix}_rssi_vs_channel.png"
    plt.savefig(rssi_plot_path)
    plt.close()
    print(f"Saved RSSI plot: {rssi_plot_path}")

    # 2. CSI Subcarrier Magnitude Plot for Phase 2 (TEST_A_TX)
    test_a_records = [r for r in records if r.get('phase') == 2 and r.get('mode') == 1 and r.get('csi_mag')]
    if test_a_records:
        plt.figure(figsize=(12, 6))
        for r in test_a_records[:5]: # Plot first few channels
            ch = r['ch']
            ant = r['ant']
            mags = r['csi_mag']
            bins = list(range(len(mags)))
            plt.plot(bins, mags, label=f"Ch {ch} ({ant})")

        plt.title("CSI Subcarrier Magnitude Spectrum (Test Antenna TX)")
        plt.xlabel("CSI Subcarrier Bin Index")
        plt.ylabel("Magnitude")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        csi_plot_path = f"{output_prefix}_csi_spectrum.png"
        plt.savefig(csi_plot_path)
        plt.close()
        print(f"Saved CSI spectrum plot: {csi_plot_path}")


def main():
    parser = argparse.ArgumentParser(description="Quad ESP32 Antenna Test Results Integration Script")
    parser.add_argument("--port", "-p", help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--file", "-f", help="Read raw log file instead of live serial port")
    parser.add_argument("--csv", default="antenna_test_results.csv", help="Output CSV filename")
    parser.add_argument("--json", default="antenna_test_results.json", help="Output JSON filename")
    parser.add_argument("--plot", action="store_true", help="Generate analysis plots")
    parser.add_argument("--duration", type=int, help="Capture duration in seconds for live serial")

    args = parser.parse_args()

    if args.file:
        records = parse_file(args.file, args.csv, args.json)
    elif args.port:
        records = process_serial(args.port, args.baud, args.csv, args.json, args.duration)
    else:
        print("Please specify either a live serial port (--port) or an input log file (--file).")
        parser.print_help()
        sys.exit(1)

    if args.plot:
        generate_plots(records)


if __name__ == "__main__":
    main()
