# CA* Pathfinding

This project implements and compares two pathfinding algorithms: A* and Canonical A* (CA*) on grid-based maps. It processes maps and scenarios from the Dragon Age 2 dataset, evaluating performance metrics such as execution time, node expansions, generations, and path quality.

## Project Structure

- `main.cpp`: The main source file containing the implementation of A* and CA* algorithms, grid management, and scenario processing.
- `dragon_Age_2/da2-map/`: Directory containing map files (`.map`).
- `dragon_Age_2/da2-scen/`: Directory containing scenario files (`.map.scen`).
- `dragon_Age_2/output/`: Directory where output files (CSV results, scenario details, and failed scenarios) are saved.

## Prerequisites

- **C++ Compiler**: A C++17 compatible compiler (e.g., Clang++, g++).
- **Operating System**: The code has been tested on macOS, but it should work on Linux and Windows with minor adjustments.
- **Standard Library**: The code uses the C++ Standard Library, including `<filesystem>` for directory operations.

## Build Instructions

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Compile the Code**:
   Use a C++17 compatible compiler. For example, with Clang++:
   ```bash
   clang++ -std=c++17 -g main.cpp -o jps_pathfinding
   ```
   - `-std=c++17`: Enables C++17 features.
   - `-g`: Includes debugging information (optional).
   - Replace `clang++` with `g++` if using GCC.

3. **Ensure Map and Scenario Files**:
   Place the `dragon_Age_2` directory with `da2-map` and `da2-scen` subdirectories in the same directory as the executable, or update the paths in `main.cpp`:
   ```cpp
   const std::string mapFolder = "<path-to-da2-map>";
   const std::string scenFolder = "<path-to-da2-scen>";
   const std::string outputFolder = "<path-to-output>";
   ```

## Run Instructions

1. **Execute the Program**:
   After compiling, run the executable:
   ```bash
   ./jps_pathfinding
   ```

2. **Input**:
   - The program expects map files in `dragon_Age_2/da2-map/` and scenario files in `dragon_Age_2/da2-scen/`.
   - Maps should be in the format expected by the Moving AI Lab (e.g., `type octile`, followed by `height`, `width`, and the grid).

3. **Output**:
   - **Detailed Results**: Saved to `dragon_Age_2/output/detailed_results.csv` with columns for scenario, weight, algorithm, time, expansions, generations, open list size, quality, and path length.
   - **Scenario Details**: For each map, a `<mapName>.map.scen.details.txt` file in `dragon_Age_2/output/` contains detailed path information and directions.
   - **Failed Scenarios**: Saved to `dragon_Age_2/output/failed_scenarios.csv` with details of scenarios where no path was found.
   - **Console Output**: Displays progress, scenario details, path directions, and failure counts.

## Notes

- **Map and Scenario Files**: The program is designed to work with the Dragon Age 2 dataset from the Moving AI Lab. Ensure these files are correctly formatted and accessible.
- **Performance Metrics**: The program evaluates A* and CA* with weights `{1.0, 2.0, 5.0, 10.0}`. Metrics include execution time (ms), node expansions, generations, open list size, and path quality (path length / optimal length).
- **Error Handling**: The program logs errors for invalid maps, scenarios, or inaccessible files. Check the console and output files for details.
- **Customizing Scenarios**: The program selects 10 scenarios per map (3 from the first 10, 3 from the middle 10, 3 from the last 10, and 1 random). Modify the `loadScenarios` function to change this behavior.
- **Cross-Platform**: On Windows, you may need to adjust the filesystem paths in `main.cpp` (e.g., use backslashes or raw strings).

## Example Output

For a map `den101d.map` with a scenario from `(10, 20)` to `(30, 40)`:
- Console:
  ```
  Processing map 1/5: den101d
  Scenario 1/10 (10,20) to (30,40)
  Running A* from (10,20) to (30,40)...
  Path found! Time: 1.234 ms, Expansions: 150, Generations: 300
  Path length: 420
  Path directions (15 steps):
  Start -> East -> Southeast -> South -> ... -> Goal
  ```
- `output/den101d.map.scen.details.txt`:
  ```
  Scenario: map = den101d.map, start = (10,20), goal = (30,40)
  Algorithm: A*
  Weight: 1.0
  Time: 1.234 ms
  Expansions: 150
  Generations: 300
  Path length: 420
  Quality: 1.050
  Path directions (15 steps):
  Start -> East -> Southeast -> South -> ... -> Goal
  ```
- `output/detailed_results.csv`:
  ```
  Scenario,Weight,Algorithm,Time (ms),Expansions,Generations,Open,Quality,Path
  1/10 (10,20) to (30,40),1.0,A*,1.234,150,300,50,1.050,420
  ```

## Contributing

Feel free to fork the repository, make improvements, and submit pull requests. Issues and feature requests can be reported on the GitHub issue tracker.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.