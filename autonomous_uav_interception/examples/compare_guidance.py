from src.simulation.pursuit_evasion import run_simulation


for method in ("pure", "predictive"):
    result = run_simulation(method=method, seed=3)

    print(f"\n{method.upper()} GUIDANCE")
    for key, value in result["metrics"].items():
        print(f"{key}: {value}")
