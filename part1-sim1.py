import os
import subprocess
import matplotlib.pyplot as plt

# Define simulation parameters
num_bss = 4  # Number of BSS
stas_per_bss = 10  # Number of STAs per BSS
simulation_time = 10  # Simulation time in seconds
phy_mode = "OfdmRate54Mbps"
output_dir = "simulation_results"

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

# Metrics containers
throughputs = []
delays = []
fairnesses = []

# Function to parse results from stdout or log files
def parse_results(sim_output):
    """Parse throughput, delay, and fairness from simulation output."""
    throughput = 0
    delay = 0
    fairness = 0
    for line in sim_output.splitlines():
        if "Throughput" in line:
            throughput = float(line.split(":")[1].strip().split(" ")[0])
        elif "Delay" in line:
            delay = float(line.split(":")[1].strip().split(" ")[0])
        elif "Fairness" in line:
            fairness = float(line.split(":")[1].strip().split(" ")[0])
    return throughput, delay, fairness

# Run simulations for different configurations
for bss_count in range(2, num_bss + 1):
    command = [
        "./ns3", "run",
        f"multi-bss --apNodes={bss_count} --networkSize={stas_per_bss} --duration={simulation_time} --phyMode={phy_mode}"
    ]
    try:
        print(f"Running simulation for {bss_count} BSS...")
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        sim_output = result.stdout
        print("simulation Output:\n", sim_output)
        throughput, delay, fairness = parse_results(sim_output)
        throughputs.append(throughput)
        delays.append(delay)
        fairnesses.append(fairness)
    except subprocess.CalledProcessError as e:
        print(f"Error during simulation: {e}")
        continue

# Plot the results
plt.figure()
plt.plot(range(1, num_bss + 1), throughputs, marker="o", label="Throughput (Mbps)")
plt.plot(range(1, num_bss + 1), delays, marker="s", label="Delay (ms)")
plt.plot(range(1, num_bss + 1), fairnesses, marker="^", label="Fairness Index")
plt.xlabel("Number of BSS")
plt.ylabel("Metrics")
plt.legend()
plt.title("Performance Metrics vs Number of BSS")
plt.grid()
plt.savefig(os.path.join(output_dir, "performance_metrics.png"))
plt.show()

print("Simulation completed. Results saved to:", output_dir)
