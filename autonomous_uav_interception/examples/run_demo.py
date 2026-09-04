import matplotlib.pyplot as plt

from src.simulation.pursuit_evasion import run_simulation


result = run_simulation(method="predictive")

print("Metrics")
for key, value in result["metrics"].items():
    print(f"{key}: {value}")

pursuer = result["pursuer"]
target = result["target"]
estimates = result["estimates"]

plt.figure()
plt.plot(pursuer[:, 0], pursuer[:, 1], label="Pursuer UAV")
plt.plot(target[:, 0], target[:, 1], label="Target UAV")
plt.plot(
    estimates[:, 0],
    estimates[:, 1],
    linestyle="--",
    label="Kalman estimate",
)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Autonomous UAV Pursuit / Rendezvous")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()

plt.figure()
plt.plot(result["time"], result["separation"])
plt.xlabel("Time [s]")
plt.ylabel("Separation [m]")
plt.title("Relative Separation")
plt.grid(True)
plt.show()
