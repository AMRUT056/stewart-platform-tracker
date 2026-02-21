# /// script
# requires-python = ">=3.8"
# dependencies = [
#     "matplotlib",
#     "numpy"
# ]
# ///

import matplotlib.pyplot as plt
import glob
import os
import csv
import re

def get_latest_files_by_type(directory="csv_outputs"):
    """Groups CSVs by prefix and returns the newest file for each category."""
    if not os.path.exists(directory):
        return {}

    files = glob.glob(os.path.join(directory, "*.csv"))
    latest_files = {}

    for filepath in files:
        basename = os.path.basename(filepath)
        # Extract prefix: "alldata_170000.csv" -> "alldata"
        file_type = re.sub(r'[\d\.]+', '', basename).replace('csv', '').rstrip('_')
        if not file_type:
            file_type = "unknown_run"

        if file_type not in latest_files:
            latest_files[file_type] = filepath
        elif os.path.getctime(filepath) > os.path.getctime(latest_files[file_type]):
            latest_files[file_type] = filepath

    return latest_files

def parse_multi_source_csv(filepath):
    """
    Dynamically groups CSV columns by DOF and forces robust command parsing.
    """
    targets = {'x': [], 'y': [], 'z': [], 'r': [], 'p': [], 'y_ang': []}
    actuals = {'x': {}, 'y': {}, 'z': {}, 'r': {}, 'p': {}, 'y_ang': {}}
    timestamps = []

    print(f"  -> Parsing: {os.path.basename(filepath)}")
    
    with open(filepath, 'r', newline='') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        if not headers: return None, None, None

        # --- 1. Map Headers to DOFs ---
        cmd_col = None
        for h in headers:
            hu = h.upper()
            if hu in ['COMMAND', 'CMD', 'INPUT', 'TARGET']:
                cmd_col = h
            elif hu.endswith('_X') or hu == 'X': actuals['x'][h] = []
            elif hu.endswith('_Y') or hu == 'Y': actuals['y'][h] = []
            elif hu.endswith('_Z') or hu == 'Z': actuals['z'][h] = []
            elif hu.endswith('_ROLL') or hu == 'ROLL': actuals['r'][h] = []
            elif hu.endswith('_PITCH') or hu == 'PITCH': actuals['p'][h] = []
            elif hu.endswith('_YAW') or hu == 'YAW': actuals['y_ang'][h] = []

        # --- 2. Extract Data Row by Row ---
        for i, row in enumerate(reader):
            # Parse Target Command (Robust against commas, brackets, and quotes)
            cmd_str = row[cmd_col] if cmd_col and cmd_col in row else ""
            if cmd_str:
                cmd_clean = cmd_str.replace(',', ' ').replace('[', '').replace(']', '').replace('"', '')
                t_vals = [float(x) for x in cmd_clean.split()]
            else:
                t_vals = []
                
            # Pad with 0.0 if command is missing or incomplete
            while len(t_vals) < 6: t_vals.append(0.0)

            targets['x'].append(t_vals[0])
            targets['y'].append(t_vals[1])
            targets['z'].append(t_vals[2])
            targets['r'].append(t_vals[3])
            targets['p'].append(t_vals[4])
            targets['y_ang'].append(t_vals[5])

            # Extract every mapped column for the actuals
            for dof, cols in actuals.items():
                for col_name in cols:
                    try:
                        val = float(row[col_name])
                    except (ValueError, TypeError):
                        val = float('nan')
                    actuals[dof][col_name].append(val)
            
            timestamps.append(i)

    return timestamps, targets, actuals

def plot_multi_source_dashboard(timestamps, targets, actuals, file_type):
    """Plots the 6-panel grid, overlaying all data sources for each DOF."""
    fig, axs = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f'Multi-Source 6-DOF Comparison: [{file_type.upper()}]', fontsize=18, fontweight='bold')

    plots = [
        ('x', 'X Translation (mm)', 0, 0),
        ('y', 'Y Translation (mm)', 0, 1),
        ('z', 'Z Translation (mm)', 0, 2),
        ('r', 'Roll (Degrees)', 1, 0),
        ('p', 'Pitch (Degrees)', 1, 1),
        ('y_ang', 'Yaw (Degrees)', 1, 2)
    ]

    for key, title, r, c in plots:
        ax = axs[r, c]
        
        # 1. ALWAYS Plot Target Command (even if it's a flat zero line)
        if targets[key]: # Make sure list isn't empty
            ax.plot(timestamps, targets[key], color='black', linestyle='--', label='TARGET', linewidth=2.5, zorder=5)
            
        # 2. Plot all available measured sources for this DOF
        has_data = False
        for source_name, data in actuals[key].items():
            if any(not __import__('math').isnan(v) for v in data):
                has_data = True
            
            # Styling: Fused is thick/solid, raw sensors are thinner/transparent
            linewidth = 2.5 if 'FUSED' in source_name.upper() else 1.5
            alpha = 1.0 if 'FUSED' in source_name.upper() else 0.6
            
            ax.plot(timestamps, data, label=source_name, linewidth=linewidth, alpha=alpha)
        
        ax.set_title(title, fontsize=12, fontweight='semibold')
        ax.grid(True, linestyle=':', alpha=0.6)
        
        if r == 1: ax.set_xlabel('Sample #')
        if c == 0: ax.set_ylabel('Value')
        
        # Only show legend if there is data to show
        if has_data or targets[key]:
            ax.legend(loc='best', fontsize=8)

    plt.tight_layout()
    
    save_path = f"csv_outputs/multisource_graph_{file_type}.png"
    plt.savefig(save_path, dpi=150)
    print(f"  -> Saved graph to: {save_path}")
    
    plt.show(block=False)

def main():
    print("Scanning for test logs...")
    latest_files = get_latest_files_by_type('csv_outputs')
    
    if not latest_files:
        print("No CSV files found in 'csv_outputs/'.")
        return

    print(f"Found {len(latest_files)} distinct log types. Generating layered dashboards...")
    
    for file_type, filepath in latest_files.items():
        print(f"\nProcessing Group: {file_type.upper()}")
        t, targets, actuals = parse_multi_source_csv(filepath)
        
        if t:
            plot_multi_source_dashboard(t, targets, actuals, file_type)
        else:
            print("  -> Skipped: File is empty or headers are missing.")

    print("\nAll graphs generated! Close the windows to exit.")
    plt.show()

if __name__ == "__main__":
    main()