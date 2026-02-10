from FeasibiltyCheck import SolutionFeasibility
from CalCulateTotalArrivalTime import CalCulateTotalArrivalTime
import numpy as np


# -------------------------------------------
filename = "Truck_Drone_Contest_new.txt"
n_drones = 2 #fixed
drone_capacity = 1 #fixed
depot_index=0 #fixed

example_solution_str = ("0,0,|-1|-1|-1")  
# ---------------------------------------------------------------------------
def parse_solution(values: str):
    """
    values: string like "1,2,3,|,10,20,|,5,6,|,100,200"
            or "1,2,3|10,20|5,6|100,200"
    returns: {"part1": [...], "part2": [...], "part3": [...], "part4": [...]}
    """

    if not isinstance(values, str):
        raise TypeError(f"Expected string from Go, got {type(values)}")

    s = values.strip()

    # normalize around pipes: ",|," or ",|" or "|," -> "|"
    s = s.replace(",|,", "|").replace(",|", "|").replace("|,", "|")

    parts = s.split("|")
    if len(parts) != 4:
        raise ValueError(f"Expected 4 parts separated by '|', got {len(parts)} parts: {parts}")

    def parse_int_list(part: str):
        # split by comma, ignore empty chunks (can happen after normalization)
        chunks = [c.strip() for c in part.split(",") if c.strip() != ""]
        return [int(c) for c in chunks]

    return {
        "part1": parse_int_list(parts[0]),
        "part2": parse_int_list(parts[1]),
        "part3": parse_int_list(parts[2]),
        "part4": parse_int_list(parts[3]),
    }



def read_data(filename: str):

    truck_times = []
    drone_times = []
    hash_count=0
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if line[0]!="#":
               row = list(map(float, line.strip().split()))
               if hash_count == 1:
                  n_customers = int(row[0])
               if hash_count == 2:
                  flight_range = int(row[0])
               elif hash_count == 3:
                  truck_times.append(row)
               elif hash_count == 4:
                  drone_times.append(row) 
            else:
               hash_count+=1
            
    n_nodes = n_customers+1 
    truck_times=np.array(truck_times)
    drone_times=np.array(drone_times)

    return n_nodes, n_customers, n_drones, flight_range, truck_times, drone_times, flight_range,  drone_capacity


n_nodes, n_customers, n_drones, flight_range, truck_times, drone_times, flight_range, drone_capacity = read_data(filename)

# -------------------------------------------

class SolutionRunner(CalCulateTotalArrivalTime, SolutionFeasibility):
    def __init__(
        self,
        solution: dict,
        truck_times,
        flight_time_matrix,
        flight_range_limit: float,
        depot_index: int = depot_index,
        max_iterations: int = 10,
        convergence_threshold: float = 1.0,
        n_drones: int = n_drones,
    ):
        """
        Wraps feasibility check + total cost calculation in one object.
        """
        self.solution = solution
        self.truck_times = truck_times
        self.drone_times = drone_times
        self.flight_time_matrix = flight_time_matrix
        self.flight_range  = flight_range 
        self.depot_index = depot_index
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.n_drones = n_drones

        n_nodes = truck_times.shape[0]

        # Create feasibility checker based on the instance
        self.feasibility = SolutionFeasibility(
            n_nodes=n_nodes,
            n_drones=n_drones,
            depot_index=depot_index,
            drone_times=flight_time_matrix,
            flight_range=flight_range_limit,
        )

    def run(self):
        """
        1) Check feasibility
        2) If infeasible: print message and return None
        3) If feasible: compute total waiting/arrival time and return it
        """
        sol = self.solution
        f = self.feasibility

        #print("=== FEASIBILITY CHECK ===")
        truck_ok = f.is_truck_route_feasible(sol)
        complete_ok = f.is_complete_solution(sol)
        parts_ok = f.are_parts_consistent(sol)
        drones_ok = f.are_all_drone_trips_feasible(sol)
        global_ok = f.is_solution_feasible(sol)

        print("Truck feasible   :", truck_ok)
        print("Complete         :", complete_ok)
        print("Parts consistent :", parts_ok)
        print("Drone trips OK   :", drones_ok)

        if not global_ok:
            #print("\nCannot calculate the total cost, since the solution is not feasible.")
            print("GLOBAL FEASIBLE  :", global_ok)
            return {'error': '', 'feasible': False, 'objective': 0.0} 

        # If we get here, the solution is feasible → compute total waiting time
        total, arr, dep, feas = self.calculate_total_waiting_time(
            solution=sol
        )
        
        if not feas:
           print("GLOBAL FEASIBLE  :", feas) 
           #print("Flight range limits on one of the drones is not satisfied") 
           return {'error': '', 'feasible': False, 'objective': 0.0}  

        print("GLOBAL FEASIBLE  :", feas) 
        print("Total objective:", float(total))
        return {'error': '', 'feasible': True, 'objective': total}#total, arr, dep


# get solution from input arguments
solution = parse_solution(example_solution_str)

runner = SolutionRunner(
    solution=solution,
    truck_times=truck_times,
    flight_time_matrix=drone_times,
    flight_range_limit=flight_range,
    depot_index=depot_index,
    max_iterations=10,
    convergence_threshold=1.0,
    n_drones=n_drones,
)

result = runner.run()  # prints feasibility and total arrival time if feasible
