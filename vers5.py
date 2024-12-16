import os
import subprocess
import shutil
import signal
import sys
from datetime import datetime
import matplotlib.pyplot as plt

def control_c(signum, frame):
    print("Exiting...")
    sys.exit(1)

signal.signal(signal.SIGINT, control_c)

def main():
    
    dirname = 'scratch'
    ns3_path = os.path.join('../../../../ns3')
    print(f"Looking for ns-3 at: {ns3_path}")

    # Check if the ns3 executable exists
    if not os.path.exists(ns3_path):
        print("Please run this program from within the correct directory.")
        sys.exit(1)

    results_dir = os.path.join(os.getcwd(), 'results', f"{dirname}-{datetime.now().strftime('%Y%m%d-%H%M%S')}")
    os.makedirs(results_dir, exist_ok=True)

    # Move to ns3 top-level directory
    os.chdir('../../../../')

    # Check for existing data files and prompt for removal
    check_and_remove('multi-bss.dat')

    # Experiment parameters
    rng_run = 1
    fixed_cca_pd = -75  # Fixed CCA-PD value
    min_lambda = -4
    max_lambda = -1
    step_size = 1
    lambdas = []
    bss_range = range(2, 6)  # Iterating over BSSs 2 to 5

    # Data storage for plotting
    aggregate_throughput = {bss: [] for bss in bss_range}

    # Run the ns3 simulation for each BSS and λ
    for bss_count in bss_range:
        for lam in range(min_lambda, max_lambda + 1, step_size):
            lambda_val = 10 ** lam
            lambdas.append(lambda_val) if lam == min_lambda else None
            cmd = (f"./ns3 run 'multi-bss-throughput-updated --rngRun={rng_run} "
                   f"--numBSS={bss_count} --ccaPdThreshold={fixed_cca_pd} "
                   f"--trafficArrivalRate={lambda_val}'")
            subprocess.run(cmd, shell=True)

        # Read throughput from generated data file
        with open('multi-bss.dat', 'r') as f:
            lines = f.readlines()
            for line in lines:
                tokens = line.split(',')
                if len(tokens) > 1:  # Assuming format: <lambda>,<throughput>
                    aggregate_throughput[bss_count].append(float(tokens[1]))

    # Plot the results
    plt.figure()
    for bss_count, throughput in aggregate_throughput.items():
        plt.plot(lambdas, throughput, marker='o', label=f'{bss_count} BSSs')
    plt.title('Aggregate Throughput vs Traffic Arrival Rate')
    plt.xlabel('Traffic Arrival Rate (λ)')
    plt.ylabel('Aggregate Throughput (Mbps)')
    plt.xscale('log')
    plt.grid()
    plt.legend()
    plot_path = os.path.join(results_dir, 'multi-bss-throughput.png')
    plt.savefig(plot_path)

    # Move result files to the experiment directory
    move_file('multi-bss.dat', results_dir)

    # Save the git commit information
    with open(os.path.join(results_dir, 'git-commit.txt'), 'w') as f:
        commit_info = subprocess.run(['git', 'show', '--name-only'], stdout=subprocess.PIPE)
        f.write(commit_info.stdout.decode())

def check_and_remove(filename):
    if os.path.exists(filename):
        response = input(f"Remove existing file {filename}? [Yes/No]: ").strip().lower()
        if response == 'yes':
            os.remove(filename)
            print(f"Removed {filename}")
        else:
            print("Exiting...")
            sys.exit(1)

def move_file(filename, destination_dir):
    if os.path.exists(filename):
        shutil.move(filename, destination_dir)

if __name__ == "__main__":
    main()
