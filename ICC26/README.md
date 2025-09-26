# ICC26 – DetNet-6TiSCH

This folder contains code, experiment setups, and data specific to our ICC 2026 paper  
**"Energy-Latency Trade-offs in 6TiSCH: Comparing Joint and Separate Routing and Scheduling"**.  

The dataset collected for the experiments is also available on Zenodo:  
👉 https://doi.org/10.5281/zenodo.17177464  

## 📂 Structure

- `models/` – formulations of the three optimization approaches (JRaS, Sep-TX, Sep-Energy).  
- `environment/` – network topologies and flow definitions.  
- `benchmarks/` – setups for Experiment 1 and Experiment 2.  

## 🚀 Getting Started

1. Install the required dependencies (see [`requirements.txt`](./requirements.txt)).  
2. Run one of the two experiment scripts from the `benchmarks/` folder.  
   The results will be stored automatically in the `data/` directory.
