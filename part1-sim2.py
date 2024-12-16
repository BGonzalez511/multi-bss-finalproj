import os
import subprocess
import matplotlib.pyplot as plt

# Define simulation parameters
cca_thresholds = range(-82, -61, 2)  # CcaSensitivity values from -82dBm to -62dBm in steps of 2
distances = [5, 15, 30]  # Distances between APs
num_bss_values = [2, 3, 4]  # Number of BSS configurations
stas_per_bss = 10  # Number of STAs per BSS
simulation_time = 10  # Simulation time in seconds
phy_mode = "OfdmRate54Mbps"
output_dir = "simulation_results"

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

# Logs
error_log_file = os.path.join(output_dir, "error_log.txt")

# Function to parse results from stdout
def parse_results(sim_output):
    """Parse throughput, delay, and fairness from simulation output."""
    throughput = None
    delay = None
    fairness = None
    for line in sim_output.splitlines():
        if "Throughput" in line:
            throughput = float(line.split(":")[1].strip().split(" ")[0])
        elif "Delay" in line:
            delay = float(line.split(":")[1].strip().split(" ")[0])
        elif "Fairness" in line:
            fairness = float(line.split(":")[1].strip().split(" ")[0])
    return throughput, delay, fairness

# Results container
results = {
    bss: {distance: {"throughputs": [], "delays": [], "fairnesses": []} for distance in distances}
    for bss in num_bss_values
}

# Run simulations for each combination of BSS, distance, and ccaSensitivity
with open(error_log_file, "w") as error_log:
    for num_bss in num_bss_values:
        print(f"Simulating for {num_bss} BSS...")
        for distance in distances:
            print(f"  AP Distance = {distance}m")
            for cca in cca_thresholds:
                command = [
                    "./ns3", "run",
                    f"multi-bss --apNodes={num_bss} --networkSize={stas_per_bss} --duration={simulation_time} "
                    f"--phyMode={phy_mode} --CcaSensitivity={cca} --distanceBetweenAps={distance}"
                ]
                try:
                    print(f"    Running simulation for CCA={cca} dBm...")
                    result = subprocess.run(command, capture_output=True, text=True, check=True)
                    sim_output = result.stdout
                    if "Simulation Completed" in sim_output:
                        throughput, delay, fairness = parse_results(sim_output)
                        results[num_bss][distance]["throughputs"].append(throughput or 0)
                        results[num_bss][distance]["delays"].append(delay or 0)
                        results[num_bss][distance]["fairnesses"].append(fairness or 0)
                    else:
                        raise RuntimeError("Simulation did not indicate completion.")
                except subprocess.CalledProcessError as e:
                    error_log.write(f"Error for BSS={num_bss}, Distance={distance}, CCA={cca}:\n{e.stderr}\n\n")
                    print(f"    Warning: Error during simulation for CCA={cca}. Logged to {error_log_file}.")
                    results[num_bss][distance]["throughputs"].append(0)
                    results[num_bss][distance]["delays"].append(0)
                    results[num_bss][distance]["fairnesses"].append(0)
                except RuntimeError as e:
                    error_log.write(f"Error for BSS={num_bss}, Distance={distance}, CCA={cca}:\n{str(e)}\n\n")
                    print(f"    Warning: Simulation output incomplete for CCA={cca}. Logged to {error_log_file}.")
                    results[num_bss][distance]["throughputs"].append(0)
                    results[num_bss][distance]["delays"].append(0)
                    results[num_bss][distance]["fairnesses"].append(0)

# Plot results
for metric, ylabel in [("throughputs", "Throughput (Mbps)"), ("delays", "Delay (ms)"), ("fairnesses", "Fairness Index")]:
    for num_bss in num_bss_values:
        plt.figure()
        for distance in distances:
            # Filter out None or invalid values for plotting
            valid_cca = [
                cca for cca, value in zip(cca_thresholds, results[num_bss][distance][metric]) if value > 0
            ]
            valid_values = [value for value in results[num_bss][distance][metric] if value > 0]
            plt.plot(
                valid_cca,
                valid_values,
                marker="o",
                label=f"Distance={distance}m"
            )
        plt.xlabel("CCA Sensitivity (dBm)")
        plt.ylabel(ylabel)
        plt.title(f"{ylabel} vs CCA Sensitivity ({num_bss} BSS)")
        plt.legend()
        plt.grid()
        metric_file = f"{metric}_vs_ccaSensitivity_{num_bss}BSS.png"
        plt.savefig(os.path.join(output_dir, metric_file))
        print(f"Plot saved: {metric_file}")

print(f"Simulation completed. Results and plots saved to: {output_dir}")
print(f"Error log saved to: {error_log_file}")
